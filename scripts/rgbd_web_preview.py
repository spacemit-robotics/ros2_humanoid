#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Serve a bounded RGB-D snapshot to the operator board, not to public clients."""

import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image

from view_hiking_depth import colorize


class Camera(Node):
    """Subscribe only while snapshots are requested; never queue a video stream."""

    def __init__(self, topics):
        super().__init__("rgbd_web_preview")
        self.topics = topics
        self.subs = []
        self.frames = {}
        self.bridge = CvBridge()
        self.last_request = 0.0
        self.last_encoded = 0.0
        self.cached = None

    def idle(self):
        if self.subs and time.monotonic() - self.last_request > 2:
            for sub in self.subs:
                self.destroy_subscription(sub)
            self.subs.clear()
            self.frames.clear()
            self.cached = None

    def snapshot(self):
        self.last_request = time.monotonic()
        if self.cached and self.last_request - self.last_encoded < 0.18:
            return self.cached
        if not self.subs:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            for key, topic in enumerate(self.topics):
                def receive(msg, index=key):
                    self.frames[index] = msg
                self.subs.append(self.create_subscription(Image, topic, receive, qos))
        self.frames.clear()
        end = self.last_request + 0.8
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.04)
            if len(self.frames) != 2:
                continue
            rgb, depth = self.frames[0], self.frames[1]
            stamps = [Time.from_msg(m.header.stamp).nanoseconds for m in (rgb, depth)]
            age = self.get_clock().now().nanoseconds - min(stamps)
            if abs(stamps[0] - stamps[1]) > 50_000_000 or not 0 <= age <= 400_000_000:
                continue
            bgr = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
            if depth.encoding != "16UC1":
                return None
            mm = self.bridge.imgmsg_to_cv2(depth, "passthrough")
            size = (320, 180)
            rgb_panel = cv2.resize(bgr, size, interpolation=cv2.INTER_AREA)
            depth_panel = cv2.resize(mm, size, interpolation=cv2.INTER_NEAREST)
            panels = np.hstack((rgb_panel, colorize(depth_panel, 2500)))
            ok, jpeg = cv2.imencode(".jpg", panels, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                self.cached = jpeg.tobytes()
                self.last_encoded = time.monotonic()
                return self.cached
        return None


class Handler(BaseHTTPRequestHandler):
    """The fixed peer can read images; there are no camera control endpoints."""

    def do_GET(self):
        if self.path != "/frame.jpg":
            self.send_error(404)
            return
        try:
            jpeg = self.server.camera.snapshot()
        except (CvBridgeError, ValueError, cv2.error) as error:
            self.server.camera.get_logger().warning(str(error), throttle_duration_sec=5)
            jpeg = None
        if not jpeg:
            self.send_error(503)
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(jpeg)

    def log_message(self, fmt, *args):
        pass  # Polling is frequent; do not journal every frame.


class PreviewServer(HTTPServer):
    def verify_request(self, request, client_address):
        request.settimeout(1)
        return client_address[0] == self.peer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", required=True)
    parser.add_argument("--peer", required=True, help="Operator board IP on the camera LAN")
    parser.add_argument("--port", type=int, default=5562)
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    args = parser.parse_args()
    cv2.setNumThreads(1)
    rclpy.init()
    camera = Camera((args.color_topic, args.depth_topic))
    try:
        with PreviewServer((args.bind, args.port), Handler) as server:
            server.camera, server.peer, server.timeout = camera, args.peer, 0.25
            while rclpy.ok():
                server.handle_request()
                camera.idle()
    finally:
        camera.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
