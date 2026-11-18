#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""View the native Hiking ZMQ depth stream without joining the control path.

Requires python3-zmq, python3-numpy and python3-opencv; no ROS environment needed.
"""

import argparse
import json
import math
import os
import struct
import sys
import time

import cv2
import numpy as np
import zmq


WINDOW = "Hiking ZMQ depth (Q / Esc to quit)"
WIDTH, HEIGHT = 64, 36


def decode_packet(packet):
    """Decode one native, single-part depth message; preserve image orientation."""
    if len(packet) < 4:
        raise ValueError("short depth packet")
    size, = struct.unpack_from("<I", packet)
    if size > 4096 or len(packet) != 4 + size + WIDTH * HEIGHT * 4:
        raise ValueError("depth packet length mismatch")
    header = json.loads(packet[4:4 + size])
    expected = {"version": 1, "profile": "g1-sdk-hiking-v1", "width": WIDTH,
                "height": HEIGHT, "dtype": "float32le", "unit": "m", "distance": "optical_z",
                "frame": "torso_link/rgbd_camera"}
    if not isinstance(header, dict) or any(header.get(key) != value for key, value in expected.items()):
        raise ValueError("unsupported depth profile")
    stamp, sequence = header.get("time"), header.get("sequence")
    if (not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp < 0
            or not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0
            or header.get("clock") not in ("sim", "robot")
            or not isinstance(header.get("session"), str) or not header["session"]):
        raise ValueError("invalid depth timestamp, sequence or session")
    depth = np.frombuffer(packet, dtype="<f4", offset=4 + size).reshape(HEIGHT, WIDTH)
    return header, depth


def model_depth(depth):
    """Match Hiking's single-frame preprocessing, returning clipped meters."""
    crop = depth[18:36, 16:48]
    crop = np.where(np.isfinite(crop) & (crop > 0), crop, np.float32(2.5))
    weight = math.exp(-0.5)
    kernel = np.array([weight, 1, weight], dtype=np.float32) / (1 + 2 * weight)
    blurred = cv2.sepFilter2D(crop, -1, kernel, kernel, borderType=cv2.BORDER_REFLECT_101)
    return np.clip(blurred, 0, 2.5)


def colorize(depth, maximum):
    """Map near depths to warm colors, far depths to cool colors, invalid to black."""
    valid = np.isfinite(depth) & (depth > 0)
    safe = np.where(valid, depth, maximum)
    values = np.rint(255 * (1 - np.clip(safe / maximum, 0, 1))).astype(np.uint8)
    colored = cv2.applyColorMap(values, cv2.COLORMAP_JET)
    colored[~valid] = 0
    return colored


def render_frame(depth, maximum, raw_only):
    """Render raw depth and, optionally, the model crop at equal panel sizes."""
    raw = cv2.resize(colorize(depth, maximum), (512, 288), interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(raw, (16 * 8, 18 * 8), (48 * 8 - 1, 36 * 8 - 1), (255, 255, 255), 1)
    panels = [raw]
    labels = ["Raw 64x36 | white box: model crop"]
    if not raw_only:
        panels.append(cv2.resize(colorize(model_depth(depth), maximum), (512, 288),
                                 interpolation=cv2.INTER_NEAREST))
        labels.append("Model 32x18 | filtered, clipped at 2.5 m")
    canvas = np.full((408, len(panels) * 512, 3), 24, dtype=np.uint8)
    for index, (panel, label) in enumerate(zip(panels, labels)):
        x = index * 512
        canvas[84:372, x:x + 512] = panel
        cv2.putText(canvas, label, (x + 10, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
    cv2.putText(canvas, f"Red: near | blue: >= {maximum:g} m | black: invalid raw",
                (10, 397), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
    return canvas


def view(args):
    """Connect a separate SUB socket with one latest message and bounded GUI work."""
    cv2.setNumThreads(1)
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.LINGER, 0)
    # Native packets are single-part. Conflation discards old frames for this SUB only.
    subscriber.setsockopt(zmq.CONFLATE, 1)
    subscriber.setsockopt(zmq.MAXMSGSIZE, 16384)
    subscriber.setsockopt(zmq.SUBSCRIBE, b"")
    last_header, image, received = None, None, None
    source_hz, next_draw, last_warning = 0.0, 0.0, -math.inf
    try:
        subscriber.connect(args.endpoint)
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        print(f"SUB {args.endpoint} | display <= {args.fps:g} FPS | Q / Esc / Ctrl-C to quit", flush=True)
        while True:
            now = time.monotonic()
            if now >= next_draw:
                next_draw = now + 1 / args.fps
                try:
                    packet = subscriber.recv(zmq.NOBLOCK)
                except zmq.Again:
                    packet = None
                if packet is not None:
                    try:
                        header, depth = decode_packet(packet)
                    except (ValueError, TypeError, OverflowError) as error:
                        if now - last_warning >= 1:
                            print(f"Ignored packet: {error}", file=sys.stderr)
                            last_warning = now
                    else:
                        source_hz = 0.0
                        if (last_header and header["session"] == last_header["session"]
                                and header["clock"] == last_header["clock"]
                                and header["time"] > last_header["time"]
                                and header["sequence"] > last_header["sequence"]):
                            source_hz = ((header["sequence"] - last_header["sequence"])
                                         / (header["time"] - last_header["time"]))
                        last_header, received = header, now
                        image = render_frame(depth, args.max_depth, args.raw_only)
                if image is None:
                    shown = np.full((408, 512 if args.raw_only else 1024, 3), 24, dtype=np.uint8)
                    line = "Waiting for depth publisher..."
                    detail = args.endpoint
                else:
                    shown = image.copy()
                    age = now - received
                    state = "STALE" if age > 0.2 else "LIVE"
                    line = (f"{state} | seq {last_header['sequence']} | {last_header['clock']} "
                            f"t={last_header['time']:.3f}s | source ~{source_hz:.1f} Hz")
                    detail = f"Last received {age:.2f}s ago | display <= {args.fps:g} FPS"
                cv2.putText(shown, line, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 255), 1)
                cv2.putText(shown, detail, (10, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
                cv2.imshow(WINDOW, shown)
            if cv2.waitKey(10) & 0xff in (27, ord("q"), ord("Q")):
                break
            try:
                # GTK does not implement WND_PROP_VISIBLE; AUTOSIZE works on GTK and Qt.
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_AUTOSIZE) < 0:
                    break
            except cv2.error:
                break  # GTK reports a deleted window as an error.
    finally:
        subscriber.close()
        context.term()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5560", help="Depth PUB endpoint")
    parser.add_argument("--fps", type=float, default=10, help="Maximum display FPS (default: 10)")
    parser.add_argument("--max-depth", type=float, default=2.5, help="Color range in meters (default: 2.5)")
    parser.add_argument("--raw-only", action="store_true", help="Show only the raw depth panel")
    args = parser.parse_args()
    if not math.isfinite(args.fps) or not 0 < args.fps <= 60:
        parser.error("--fps must be in (0, 60]")
    if not math.isfinite(args.max_depth) or args.max_depth <= 0:
        parser.error("--max-depth must be finite and positive")
    if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("a graphical desktop is required; run the viewer on a computer with a display")
    try:
        view(args)
    except KeyboardInterrupt:
        pass
    except (zmq.ZMQError, cv2.error) as error:
        parser.exit(1, f"Depth viewer: {error}\n")


if __name__ == "__main__":
    main()
