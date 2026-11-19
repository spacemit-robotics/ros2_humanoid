#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Bridge RealSense ROS2 depth to ZMQ, with preview and guarded control modes.

Source /opt/ros/humble/setup.bash first. Control requires a live ROS-to-robot
clock mapping produced from measured driver anchors via --clock-endpoint or --time-map.
"""

import argparse
from collections import Counter, deque
import json
import math
import os
from pathlib import Path
import struct
import sys
import time
import uuid

import numpy as np
import zmq


WIDTH, HEIGHT, FOVY = 64, 36, 58.29
CAMERA_POS = [0.0576235, 0.01753, 0.42987]
CAMERA_XYAXES = [0, -1, 0, 0.738455340625884, 0, 0.674302387583723]


def projection(info):
    """Resample rectified optical rays; this does not change camera mounting."""
    p, r = np.asarray(info.p), np.asarray(info.r)
    if (not 0 < info.width <= 8192 or not 0 < info.height <= 8192
            or p.shape != (12,) or r.shape != (9,)
            or not np.isfinite(p).all() or not np.isfinite(r).all()
            or not np.allclose(r.reshape(3, 3), np.eye(3), atol=1e-6, rtol=0)
            or not np.allclose(p[[1, 3, 4, 7, 8, 9, 11]], 0, atol=1e-8, rtol=0)
            or abs(p[10] - 1) > 1e-8):
        raise ValueError("unsupported CameraInfo: need finite, unrotated monocular rectified projection")
    fx, fy, cx, cy = p[0], p[5], p[2], p[6]
    if min(fx, fy) <= 0 or not 0 <= cx < info.width or not 0 <= cy < info.height:
        raise ValueError("invalid rectified CameraInfo intrinsics")
    focal = HEIGHT / (2 * math.tan(math.radians(FOVY / 2)))
    x, y = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
    u = np.rint((x - (WIDTH - 1) / 2) * fx / focal + cx).astype(int)
    v = np.rint((y - (HEIGHT - 1) / 2) * fy / focal + cy).astype(int)
    valid = (u >= 0) & (u < info.width) & (v >= 0) & (v < info.height)
    return u, v, valid


def project_depth(message, mapping, depth_scale):
    """Return meters of optical Z; honor endian and row padding."""
    if message.encoding not in ("16UC1", "32FC1"):
        raise ValueError(f"unsupported depth encoding: {message.encoding}")
    dtype = np.dtype((">" if message.is_bigendian else "<")
                     + ("u2" if message.encoding == "16UC1" else "f4"))
    if (message.step < message.width * dtype.itemsize
            or len(message.data) < message.step * message.height):
        raise ValueError("invalid depth image buffer/stride")
    image = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                       strides=(message.step, dtype.itemsize))
    u, v, covered = mapping
    result = np.zeros((HEIGHT, WIDTH), dtype="<f4")
    values = image[v[covered], u[covered]].astype(np.float32)
    if message.encoding == "16UC1":
        values *= depth_scale
    values[~np.isfinite(values) | (values <= 0)] = 0
    result[covered] = values
    return result


def encode(depth, header):
    """Pack one native single-part ZMQ message."""
    data = json.dumps(header, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(data) > 4096:
        raise ValueError("metadata exceeds native header size limit")
    return struct.pack("<I", len(data)) + data + depth.astype("<f4", copy=False).tobytes()


def robot_time(path, stamp, now, max_age, max_error):
    """Read an atomically replaced, live affine map; never invent a clock offset."""
    try:
        with Path(path).open(encoding="utf-8") as source:
            data = json.loads(source.read(4097))
        if (not isinstance(data, dict) or data.get("version") != 1
                or data.get("source_clock") != "ros" or data.get("target_clock") != "robot"
                or not isinstance(data.get("session"), str) or not data["session"]):
            raise ValueError("invalid time-map identity")
        names = ("source_time_s", "target_time_s", "scale", "updated_ros_time_s", "error_bound_s")
        for name in names:
            value = data.get(name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"invalid time-map field: {name}")
        age = now - data["updated_ros_time_s"]
        if not -0.02 <= age <= max_age:
            raise ValueError("time map expired or from the future")
        if (not 0.999 <= data["scale"] <= 1.001 or data["target_time_s"] < 0
                or not 0 <= data["error_bound_s"] <= max_error
                or abs(data["source_time_s"] - data["updated_ros_time_s"]) > max_age
                or abs(stamp - data["source_time_s"]) > max_age):
            raise ValueError("time-map drift, error or anchor age exceeds limit")
        mapped = data["target_time_s"] + (stamp - data["source_time_s"]) * data["scale"]
        if not math.isfinite(mapped) or mapped < 0:
            raise ValueError("invalid mapped robot time")
        return mapped, data["session"], data["error_bound_s"]
    except (OSError, ValueError, TypeError) as error:
        raise ValueError(f"robot time unavailable: {error}") from error


class DepthState:
    """ROS-independent input validation, session lifecycle and packet construction."""

    def __init__(self, args):
        self.args = args
        self.info = self.mapping = self.info_key = None
        self.last_input = self.last_ros = self.last_stamp = self.last_output = self.last_packet = None
        self.map_session = None
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.counts = Counter()
        self.recent = deque(maxlen=300)
        self.reason = "waiting for CameraInfo and depth"
        self.stale = False
        self.coverage = self.valid_fraction = 0.0

    def reset(self, clear_source=True):
        self.session, self.sequence = uuid.uuid4().hex, 0
        self.last_output = None
        if clear_source:
            self.last_stamp = None

    def set_info(self, info):
        key = (info.width, info.height, info.header.frame_id, tuple(info.p), tuple(info.r))
        if self.info is not None and key == self.info_key:
            return
        try:
            if info.header.frame_id != self.args.expected_frame:
                raise ValueError(f"expected optical frame {self.args.expected_frame}")
            mapping = projection(info)
        except ValueError:
            self.info = self.mapping = self.info_key = None
            self.coverage = 0
            self.reset()
            raise
        self.info, self.mapping, self.info_key = info, mapping, key
        self.coverage = float(mapping[2].mean())
        self.reset()

    def check_stale(self, now):
        if self.last_input is not None and now - self.last_input > self.args.stream_timeout and not self.stale:
            self.stale = True
            self.reset()
            self.reason = "depth stream stale; waiting for fresh frames"
            return True
        return False

    def consume(self, message, ros_now, mono_now):
        """Reject invalid input; no control packets are produced on any failed check."""
        self.counts["received"] += 1
        if self.last_ros is not None and ros_now < self.last_ros - 0.02:
            self.reset()
        self.last_ros = ros_now
        if self.info is None:
            raise ValueError("waiting for valid CameraInfo")
        if ((message.width, message.height, message.header.frame_id)
                != (self.info.width, self.info.height, self.info.header.frame_id)):
            raise ValueError("depth image and CameraInfo dimensions/frame differ")
        sec, nsec = message.header.stamp.sec, message.header.stamp.nanosec
        if sec < 0 or not 0 <= nsec < 1_000_000_000:
            raise ValueError("invalid ROS acquisition timestamp")
        stamp_ns = sec * 1_000_000_000 + nsec
        stamp = stamp_ns * 1e-9
        age = ros_now - stamp
        if stamp_ns <= 0 or not -0.02 <= age <= self.args.max_frame_age:
            raise ValueError("depth acquisition time stale or from the future")
        self.check_stale(mono_now)
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            self.counts["duplicate" if stamp_ns == self.last_stamp else "out_of_order"] += 1
            return None
        self.last_stamp, self.last_input, self.stale = stamp_ns, mono_now, False
        self.recent.append((mono_now, age))
        depth = project_depth(message, self.mapping, self.args.depth_scale)
        self.valid_fraction = float(np.mean(depth > 0))
        header = {
            "version": 1, "profile": "g1-realsense-preview-v1",
            "width": WIDTH, "height": HEIGHT, "dtype": "float32le", "unit": "m", "distance": "optical_z",
            "frame": message.header.frame_id, "fovy_deg": FOVY, "clock": "ros", "time": stamp,
            "control_ready": False, "source_stamp_ns": stamp_ns, "source_frame": message.header.frame_id,
            "source_size": [message.width, message.height], "source_encoding": message.encoding,
            "source_p": list(self.info.p), "source_r": list(self.info.r),
            "coverage": self.coverage, "valid_fraction": self.valid_fraction, "capture_age_s": age,
        }
        if self.args.mode == "control":
            if self.coverage != 1:
                raise ValueError("control requires complete target projection coverage")
            if self.valid_fraction == 0:
                raise ValueError("control depth is entirely invalid")
            mapped, identity, error = robot_time(self.args.time_map, stamp, ros_now,
                                                  self.args.time_map_max_age, self.args.max_time_error)
            if identity != self.map_session:
                self.reset(clear_source=False)
                self.map_session = identity
            if self.last_output is not None and mapped <= self.last_output:
                raise ValueError("mapped robot time is not increasing")
            header.update(profile="g1-sdk-hiking-v1", frame="torso_link/rgbd_camera", clock="robot",
                          time=mapped, control_ready=True, camera_pos=CAMERA_POS, camera_xyaxes=CAMERA_XYAXES,
                          time_map_session=identity, time_error_bound_s=error)
        if self.last_packet is not None and mono_now - self.last_packet > self.args.stream_timeout:
            self.reset(clear_source=False)
        self.sequence += 1
        header.update(sequence=self.sequence, session=self.session)
        packet = encode(depth, header)
        self.last_output = header["time"]
        self.last_packet = mono_now
        self.reason = "ready" if self.args.mode == "control" else "preview; robot clock not mapped"
        return packet


def create_node(args):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    class DepthBridge(Node):
        def __init__(self):
            super().__init__("realsense_depth_zmq")
            self.state = DepthState(args)
            self.zmq_context = zmq.Context()
            self.publisher = self.zmq_context.socket(zmq.PUB)
            self.publisher.setsockopt(zmq.LINGER, 0)
            self.publisher.setsockopt(zmq.SNDHWM, 2)
            try:
                self.publisher.bind(args.bind)
            except Exception:
                self.publisher.close()
                self.zmq_context.term()
                self.destroy_node()
                raise
            self.emitted = self.total_bytes = 0
            self.last_warning = -math.inf
            self.report_at, self.report_count = time.monotonic(), 0
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CameraInfo, args.camera_info_topic, self.on_info, qos)
            self.create_subscription(Image, args.depth_topic, self.on_image, qos)
            self.create_timer(0.1, self.watchdog)
            self.create_timer(5, self.report)
            self.get_logger().info(f"mode={args.mode}, ROS_DOMAIN_ID={args.domain_id}, "
                                   f"{args.depth_topic} -> {args.bind}; SDK camera mounting assumed")

        def warning(self, error):
            self.state.reason = str(error)
            self.state.counts["rejected"] += 1
            if self.context.ok() and time.monotonic() - self.last_warning >= 2:
                self.get_logger().warning(str(error))
                self.last_warning = time.monotonic()

        def on_info(self, info):
            old = self.state.info_key
            try:
                self.state.set_info(info)
                if old != self.state.info_key:
                    self.get_logger().info(f"Source {info.width}x{info.height} -> {WIDTH}x{HEIGHT}, "
                                           f"target coverage={self.state.coverage:.1%}")
            except ValueError as error:
                self.warning(error)

        def on_image(self, message):
            try:
                packet = self.state.consume(message, self.get_clock().now().nanoseconds * 1e-9, time.monotonic())
                if packet is not None:
                    self.publisher.send(packet, zmq.NOBLOCK)
                    self.emitted += 1
                    self.total_bytes += len(packet)
            except (ValueError, TypeError, BufferError, OverflowError, zmq.ZMQError) as error:
                self.warning(error)

        def watchdog(self):
            if self.state.check_stale(time.monotonic()):
                self.get_logger().warning(self.state.reason)

        def report(self):
            now = time.monotonic()
            fps = (self.emitted - self.report_count) / max(now - self.report_at, 1e-6)
            recent = [age for when, age in self.state.recent if now - when <= 5]
            age = f"{np.percentile(recent, 95) * 1000:.1f} ms" if recent else "n/a"
            text = (f"mode={args.mode}, state={self.state.reason}, queued={self.emitted}, recent={fps:.1f} FPS, "
                    f"age_p95={age}, coverage={self.state.coverage:.1%}, valid={self.state.valid_fraction:.1%}, "
                    f"counts={dict(self.state.counts)}")
            if self.context.ok():
                self.get_logger().info(text)
            else:
                print(text, flush=True)  # ROS context may already be shut down by SIGINT.
            self.report_at, self.report_count = now, self.emitted

        def close(self):
            try:
                self.publisher.close()
                self.zmq_context.term()
            finally:
                self.destroy_node()

    return DepthBridge()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="tcp://0.0.0.0:5560")
    parser.add_argument("--domain-id", type=int, default=73)
    parser.add_argument("--depth-topic", default="/camera/depth/image_rect_raw")
    parser.add_argument("--camera-info-topic", default="/camera/depth/camera_info")
    parser.add_argument("--expected-frame", default="camera_depth_optical_frame")
    parser.add_argument("--depth-scale", type=float, default=0.001, help="Meters per 16UC1 unit; 32FC1 uses meters")
    parser.add_argument("--mode", choices=("preview", "control"), default="preview")
    parser.add_argument("--time-map", help="Atomically refreshed ROS-to-robot clock map JSON (or use --clock-endpoint)")
    parser.add_argument("--clock-endpoint", help="Start integrated clock sync, e.g. tcp://192.168.60.153:5561")
    parser.add_argument("--time-map-max-age", type=float, default=2.0)
    parser.add_argument("--max-time-error", type=float, default=0.02)
    parser.add_argument("--max-frame-age", type=float, default=0.2)
    parser.add_argument("--stream-timeout", type=float, default=0.2)
    parser.add_argument("--duration", type=float, default=0, help="Exit after N seconds; 0 runs until Ctrl-C")
    args = parser.parse_args(argv)
    for name in ("depth_scale", "time_map_max_age", "max_time_error", "max_frame_age", "stream_timeout", "duration"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0 or (name != "duration" and value == 0):
            parser.error(f"--{name.replace('_', '-')} must be finite and positive (duration may be zero)")
    if not 0 <= args.domain_id <= 232:
        parser.error("--domain-id must be between 0 and 232")
    if (args.time_map_max_age > 5 or args.max_time_error > 0.02
            or args.max_frame_age > 0.2 or args.stream_timeout > 0.2):
        parser.error("time-map age must be <=5 s, error <=0.02 s, frame age and stream timeout <=0.2 s")
    if args.clock_endpoint:
        if args.mode != "control":
            parser.error("--clock-endpoint requires --mode control")
        if not args.time_map:
            args.time_map = str(Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
                                / "g1_clock_map.json")
    if args.mode == "control" and not args.time_map:
        parser.error("--mode control requires --clock-endpoint or --time-map; no zero clock offset is assumed")
    return args


def main():
    args = parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        sys.exit("Source /opt/ros/humble/setup.bash first; install python3-numpy and python3-zmq.")
    rclpy.init(args=[])
    node = sync = None
    try:
        node = create_node(args)
        if args.clock_endpoint:
            try:
                from g1_clock_sync import ClockSync
            except ImportError as error:
                raise ValueError("g1_clock_sync.py must be next to this script") from error
            sync = ClockSync(args.clock_endpoint, args.time_map, args.time_map_max_age, args.max_time_error)
            sync.start()
        started = time.monotonic()
        while rclpy.ok() and (not args.duration or time.monotonic() - started < args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
        if rclpy.ok():
            node.report()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ValueError, RuntimeError, zmq.ZMQError) as error:
        if rclpy.ok():
            print(f"Depth bridge: {error}", file=sys.stderr)
            return 1
    finally:
        if sync is not None:
            sync.close()
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
