#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Write paired BGR24 camera frames to the on-demand video encoder."""

import argparse
import sys
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
    """Use only fresh, aligned RGB-D pairs. The media server starts this node on demand."""

    def __init__(self, topics):
        super().__init__("rgbd_web_preview")
        self.topics = topics
        self.subs = []
        self.frames = {}
        self.bridge = CvBridge()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for key, topic in enumerate(self.topics):
            def receive(msg, index=key):
                self.frames[index] = msg
            self.subs.append(self.create_subscription(Image, topic, receive, qos))

    def frame(self):
        self.frames.clear()
        end = time.monotonic() + 0.5
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
            return np.hstack((rgb_panel, colorize(depth_panel, 2500)))
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    args = parser.parse_args()
    cv2.setNumThreads(1)
    rclpy.init()
    camera = Camera((args.color_topic, args.depth_topic))
    try:
        while rclpy.ok():
            try:
                frame = camera.frame()
                if frame is not None:
                    sys.stdout.buffer.write(frame.tobytes())
            except (CvBridgeError, ValueError, cv2.error) as error:
                camera.get_logger().warning(str(error), throttle_duration_sec=5)
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        camera.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
