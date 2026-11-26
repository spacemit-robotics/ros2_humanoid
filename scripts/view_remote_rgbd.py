#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""View remote ROS RGB-D images via ZeroMQ: 640x360 per panel, 10 Hz."""

import argparse
import os
import struct
import sys
import time

import cv2
import numpy as np
import zmq


HEADER = struct.Struct('!4sIQf')
MAGIC = b'RGB2'
WINDOW = 'G1 RGB-D: RGB | depth (Q / Esc to quit)'
WIDTH, HEIGHT = 640, 360


def render_pair(rgb, depth, encoding, mode, maximum):
    """Apply the same crop to aligned images and colorize depth in meters."""
    if rgb.shape[:2] != depth.shape[:2] or depth.ndim != 2:
        raise ValueError('RGB and aligned depth must have the same width and height')
    if mode == 'crop':
        height, width = depth.shape
        if width < WIDTH or height < HEIGHT:
            raise ValueError('Images smaller than 640x360; use --mode resize')
        x, y = (width - WIDTH) // 2, (height - HEIGHT) // 2
        rgb = rgb[y:y+HEIGHT, x:x+WIDTH]
        depth = depth[y:y+HEIGHT, x:x+WIDTH]
    else:
        rgb = cv2.resize(rgb, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
        depth = cv2.resize(depth, (WIDTH, HEIGHT), interpolation=cv2.INTER_NEAREST)
    if encoding == '16UC1':
        meters = depth.astype(np.float32) * 0.001
    elif encoding == '32FC1':
        meters = depth
    else:
        raise ValueError(f'Unsupported depth encoding: {encoding}')
    valid = np.isfinite(meters) & (meters > 0)
    safe = np.where(valid, meters, maximum)
    values = np.rint(255 * (1 - np.clip(safe / maximum, 0, 1))).astype(np.uint8)
    colored = cv2.applyColorMap(values, cv2.COLORMAP_JET)
    colored[~valid] = 0
    return np.hstack((rgb, colored))


def publish(args):
    """Run independently on the camera host and publish compressed images."""
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '0'
    import rclpy
    from cv_bridge import CvBridge, CvBridgeError
    import message_filters
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class Camera(Node):
        def __init__(self):
            super().__init__(f'rgbd_remote_preview_{os.getpid()}')
            self.bridge = CvBridge()
            self.latest = None
            self.color = message_filters.Subscriber(
                self, Image, args.color_topic, qos_profile=qos_profile_sensor_data)
            self.depth = message_filters.Subscriber(
                self, Image, args.depth_topic, qos_profile=qos_profile_sensor_data)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.color, self.depth], queue_size=5, slop=0.05)
            self.sync.registerCallback(self.receive)
            self.timer = self.create_timer(1 / args.fps, self.send)
            self.get_logger().info(
                f'Domain {args.domain}: {args.color_topic} + {args.depth_topic}; '
                f'{args.mode} {WIDTH}x{HEIGHT} each, depth 0-{args.max_depth:g} m; '
                f'ZeroMQ PUB tcp://0.0.0.0:{args.port}')

        def receive(self, color, depth):
            # Only keep the newest synchronized pair; never queue video frames.
            self.latest = (color, depth, time.monotonic())

        def send(self):
            pair, self.latest = self.latest, None
            now = time.monotonic()
            if pair is not None and now - pair[2] <= 0.5:
                color, depth, _ = pair
                try:
                    frame = render_pair(
                        self.bridge.imgmsg_to_cv2(color, 'bgr8'),
                        self.bridge.imgmsg_to_cv2(depth, 'passthrough'),
                        depth.encoding, args.mode, args.max_depth)
                    success, jpeg = cv2.imencode(
                        '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
                    if not success:
                        raise ValueError('JPEG encoding failed')
                except (CvBridgeError, ValueError, cv2.error) as error:
                    self.get_logger().warning(str(error), throttle_duration_sec=5)
                else:
                    stamp = color.header.stamp.sec * 1_000_000_000 + color.header.stamp.nanosec
                    payload = jpeg.tobytes()
                    # One complete image per ZMQ message: CONFLATE is not multipart-safe.
                    publisher.send(
                        HEADER.pack(MAGIC, len(payload), stamp, args.max_depth) + payload)

    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.setsockopt(zmq.CONFLATE, 1)
    publisher.setsockopt(zmq.SNDBUF, 128 * 1024)
    camera = None
    ros_initialized = False
    try:
        publisher.bind(f'tcp://0.0.0.0:{args.port}')
        rclpy.init(args=[])
        ros_initialized = True
        camera = Camera()
        rclpy.spin(camera)
    except KeyboardInterrupt:
        pass
    finally:
        if camera is not None:
            camera.destroy_node()
        if ros_initialized:
            rclpy.try_shutdown()
        publisher.close()
        context.term()


def decode_packet(packet):
    """Decode one self-contained ZMQ JPEG message, without ROS on the viewer."""
    if len(packet) < HEADER.size:
        raise ValueError('Short remote video packet')
    magic, size, stamp, maximum = HEADER.unpack_from(packet)
    if (magic != MAGIC or size > 2_000_000 or len(packet) != HEADER.size + size
            or not np.isfinite(maximum) or maximum <= 0):
        raise ValueError('Invalid remote video packet')
    frame = cv2.imdecode(
        np.frombuffer(packet, np.uint8, offset=HEADER.size), cv2.IMREAD_COLOR)
    if frame is None or frame.shape != (HEIGHT, WIDTH * 2, 3):
        raise ValueError('Invalid JPEG preview frame')
    return frame, stamp, size, maximum


def view(args):
    """Connect directly to a running ZeroMQ publisher; no ROS or SSH needed."""
    if not args.headless and sys.platform.startswith('linux') and not (
            os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise RuntimeError('A graphical desktop is required; --headless is available for diagnosis')
    endpoint = f'tcp://{args.host}:{args.port}'
    print(f'Connecting to {endpoint}; waiting for the remote publisher', flush=True)
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.LINGER, 0)
    subscriber.setsockopt(zmq.CONFLATE, 1)
    subscriber.setsockopt(zmq.RCVBUF, 128 * 1024)
    subscriber.setsockopt(zmq.MAXMSGSIZE, 2_000_000 + HEADER.size)
    subscriber.setsockopt(zmq.SUBSCRIBE, b'')
    last = None
    frame = None
    try:
        subscriber.connect(endpoint)
        if not args.headless:
            cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(WINDOW, np.zeros((HEIGHT, WIDTH * 2, 3), np.uint8))
        began = time.monotonic()
        while not args.duration or time.monotonic() - began < args.duration:
            if subscriber.poll(timeout=20):
                packet = subscriber.recv()
                frame, _stamp, _size, maximum = decode_packet(packet)
                last = time.monotonic()
                if not args.headless:
                    cv2.imshow(WINDOW, frame)
                    cv2.setWindowTitle(WINDOW, f'RGB | depth 0-{maximum:g} m | Q / Esc')
            if not args.headless:
                if cv2.waitKey(1) & 0xff in (27, ord('q'), ord('Q')):
                    break
                try:
                    if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_AUTOSIZE) < 0:
                        break
                except cv2.error:
                    break
                if last is not None and time.monotonic() - last > 1:
                    cv2.setWindowTitle(WINDOW, 'STALE: waiting for fresh RGB-D images | Q / Esc')
        if args.save_frame and frame is not None:
            if not cv2.imwrite(args.save_frame, frame):
                raise RuntimeError(f'Cannot save frame: {args.save_frame}')
        if args.duration and frame is None:
            raise RuntimeError('No images received; check the remote --publish process, '
                               f'and TCP reachability of {endpoint}')
    finally:
        subscriber.close()
        context.term()
        if not args.headless:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--publish', action='store_true',
                        help='Run the ROS publisher on the remote camera host')
    parser.add_argument('--host', default='10.0.91.110', help='Publisher address for the viewer')
    parser.add_argument('--domain', type=int, default=73, help='Publisher ROS_DOMAIN_ID')
    parser.add_argument('--port', type=int, default=5562, help='Remote ZeroMQ TCP port')
    parser.add_argument('--fps', type=float, default=10, help='Publisher transmission rate')
    parser.add_argument('--mode', choices=('crop', 'resize'), default='crop',
                        help='crop: center 640x360; resize: retain the full field of view')
    parser.add_argument('--max-depth', type=float, default=3.0, help='Depth color scale in meters')
    parser.add_argument('--quality', type=int, default=80, help='JPEG quality, 1-100')
    parser.add_argument('--color-topic', default='/camera/color/image_raw')
    parser.add_argument('--depth-topic', default='/camera/aligned_depth_to_color/image_raw')
    parser.add_argument('--duration', type=float, default=0, help='Stop after N seconds; 0 is unlimited')
    parser.add_argument('--headless', action='store_true', help='Receive without a window')
    parser.add_argument('--save-frame', help='Save the last displayed frame when exiting')
    args = parser.parse_args()
    if not 0 < args.fps <= 60 or not np.isfinite(args.fps):
        parser.error('--fps must be between 0 and 60')
    if not np.isfinite(args.max_depth) or args.max_depth <= 0:
        parser.error('--max-depth must be positive')
    if not 1 <= args.quality <= 100 or not 0 <= args.domain <= 232:
        parser.error('--quality must be 1-100 and --domain must be 0-232')
    if not 1024 <= args.port <= 65535:
        parser.error('--port must be 1024-65535')
    if not np.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be nonnegative')
    cv2.setNumThreads(1)
    try:
        publish(args) if args.publish else view(args)
    except KeyboardInterrupt:
        return 0
    except (OSError, EOFError, RuntimeError, ValueError, cv2.error, zmq.ZMQError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
