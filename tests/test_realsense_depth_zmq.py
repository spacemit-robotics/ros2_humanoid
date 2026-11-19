#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""ROS-independent tests of control gating, clock freshness and depth geometry."""

import importlib.util
from contextlib import redirect_stderr
import io
import json
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np

SPEC = importlib.util.spec_from_file_location(
    "realsense_depth_zmq", Path(__file__).resolve().parents[1] / "scripts/realsense_depth_zmq.py")
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


def info(width=640, height=360, fx=322.156677, cx=317.4133, cy=178.53789):
    return NS(width=width, height=height, header=NS(frame_id="camera_depth_optical_frame"),
              p=[fx, 0, cx, 0, 0, fx, cy, 0, 0, 0, 1, 0], r=np.eye(3).ravel().tolist())


def image(stamp=1000.0, value=2000, encoding="16UC1", big=False, padding=0):
    dtype = (">" if big else "<") + ("u2" if encoding == "16UC1" else "f4")
    data = np.full((360, 640 + padding), value, dtype=dtype)
    ns = round(stamp * 1e9)
    return NS(width=640, height=360, encoding=encoding, is_bigendian=big,
              step=data.strides[0], data=data.tobytes(),
              header=NS(frame_id="camera_depth_optical_frame", stamp=NS(sec=ns // 10**9, nanosec=ns % 10**9)))


def unpack(packet):
    size, = struct.unpack_from("<I", packet)
    return json.loads(packet[4:4 + size]), np.frombuffer(packet, "<f4", offset=4 + size).reshape(36, 64)


class DepthBridgeTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.map_path = Path(self.directory.name) / "clock.json"
        self.args = bridge.parse_args([])

    def make_state(self, control=False):
        if control:
            self.args.mode = "control"
            self.args.time_map = str(self.map_path)
        state = bridge.DepthState(self.args)
        state.set_info(info())
        return state

    def write_map(self, now=1000.0, target=40.0, **updates):
        data = dict(version=1, source_clock="ros", target_clock="robot", session="imu-boot-1",
                    source_time_s=now, target_time_s=target, scale=1.0,
                    updated_ros_time_s=now, error_bound_s=0.005)
        data.update(updates)
        self.map_path.write_text(json.dumps(data))

    def test_preview_projection_units_endian_and_stride(self):
        for encoding, value, big in [("16UC1", 2000, False), ("16UC1", 2000, True), ("32FC1", 2.0, True)]:
            with self.subTest(encoding=encoding, big=big):
                state = self.make_state()
                header, depth = unpack(state.consume(image(value=value, encoding=encoding, big=big, padding=7),
                                                    1000.03, 1.0))
                self.assertEqual(header["profile"], "g1-realsense-preview-v1")
                self.assertEqual(header["clock"], "ros")
                self.assertFalse(header["control_ready"])
                self.assertEqual(header["coverage"], 1.0)
                np.testing.assert_allclose(depth, 2.0)

    def test_control_matches_native_contract_and_maps_acquisition_time(self):
        self.write_map(scale=1.0001)
        state = self.make_state(control=True)
        header, _ = unpack(state.consume(image(1000.1), 1000.15, 1.0))
        self.assertEqual(header["profile"], "g1-sdk-hiking-v1")
        self.assertEqual(header["clock"], "robot")
        self.assertEqual(header["frame"], "torso_link/rgbd_camera")
        self.assertEqual(header["camera_pos"], bridge.CAMERA_POS)
        self.assertEqual(header["camera_xyaxes"], bridge.CAMERA_XYAXES)
        self.assertAlmostEqual(header["time"], 40.10001)
        self.assertNotAlmostEqual(header["time"], 40.150015)
        self.assertTrue(header["control_ready"])
        self.assertIsNone(state.consume(image(1000.1), 1000.16, 1.01))

    def test_missing_or_broken_map_never_falls_back_to_control(self):
        state = self.make_state(control=True)
        with self.assertRaisesRegex(ValueError, "robot time unavailable"):
            state.consume(image(), 1000.01, 1.0)
        self.map_path.write_text('{"version":')
        with self.assertRaises(ValueError):
            state.consume(image(1000.03), 1000.04, 1.03)
        self.assertEqual(state.sequence, 0)

    def test_reject_bad_mapping_fields_and_expiration(self):
        for changes in [dict(updated_ros_time_s=997), dict(updated_ros_time_s=1001),
                        dict(error_bound_s=.021), dict(error_bound_s=-1), dict(scale=1.01),
                        dict(source_time_s=995), dict(target_time_s=-1), dict(scale=float("nan")),
                        dict(scale=True), dict(session=""), dict(target_clock="unix")]:
            with self.subTest(changes=changes):
                self.write_map(**changes)
                with self.assertRaises(ValueError):
                    bridge.robot_time(self.map_path, 1000, 1000.01, 2.0, .02)

    def test_frame_age_duplicates_and_out_of_order(self):
        state = self.make_state()
        for stamp in [999.7, 1000.1]:
            with self.assertRaisesRegex(ValueError, "stale or from the future"):
                state.consume(image(stamp), 1000, 1)
        state.consume(image(), 1000.01, 1)
        session = state.session
        self.assertIsNone(state.consume(image(), 1000.02, 1.01))
        self.assertIsNone(state.consume(image(999.99), 1000.02, 1.02))
        self.assertEqual(state.session, session)
        self.assertEqual(state.counts["duplicate"], 1)
        self.assertEqual(state.counts["out_of_order"], 1)

    def test_stream_loss_recovers_with_new_history_session(self):
        state = self.make_state()
        first, _ = unpack(state.consume(image(), 1000.01, 1))
        self.assertTrue(state.check_stale(1.3))
        self.assertFalse(state.check_stale(1.4))
        second, _ = unpack(state.consume(image(1000.4), 1000.41, 1.4))
        self.assertNotEqual(first["session"], second["session"])
        self.assertEqual(second["sequence"], 1)

    def test_ros_time_reset_creates_new_session(self):
        state = self.make_state()
        first, _ = unpack(state.consume(image(), 1000.01, 1))
        second, _ = unpack(state.consume(image(10), 10.01, 1.02))
        self.assertNotEqual(first["session"], second["session"])

    def test_invalid_camera_info_clears_old_projection(self):
        state = self.make_state()
        bad = info()
        bad.p[0] = float("nan")
        with self.assertRaises(ValueError):
            state.set_info(bad)
        self.assertIsNone(state.mapping)
        with self.assertRaisesRegex(ValueError, "CameraInfo"):
            state.consume(image(), 1000.01, 1)
        state.set_info(info())
        self.assertIsNotNone(state.consume(image(), 1000.01, 1))

    def test_control_rejects_partial_coverage_or_invalid_depth(self):
        self.write_map()
        state = self.make_state(control=True)
        state.set_info(info(fx=450))
        with self.assertRaisesRegex(ValueError, "complete target"):
            state.consume(image(), 1000.01, 1)
        state.set_info(info())
        with self.assertRaisesRegex(ValueError, "entirely invalid"):
            state.consume(image(1000.03, value=0), 1000.04, 1.03)

    def test_control_resume_after_map_outage_resets_session(self):
        self.write_map()
        state = self.make_state(control=True)
        first, _ = unpack(state.consume(image(), 1000.01, 1))
        with self.assertRaisesRegex(ValueError, "expired"):
            state.consume(image(1002.1), 1002.11, 3.1)
        self.write_map(now=1002.13, target=42.13)
        second, _ = unpack(state.consume(image(1002.13), 1002.14, 3.13))
        self.assertNotEqual(first["session"], second["session"])
        self.assertEqual(second["sequence"], 1)

    def test_robot_reset_requires_new_map_identity_and_no_backwards_output(self):
        self.write_map()
        state = self.make_state(control=True)
        first, _ = unpack(state.consume(image(), 1000.01, 1))
        self.write_map(now=1000.03, target=1)
        with self.assertRaisesRegex(ValueError, "not increasing"):
            state.consume(image(1000.03), 1000.04, 1.03)
        self.write_map(now=1000.06, target=1.03, session="imu-boot-2")
        second, _ = unpack(state.consume(image(1000.06), 1000.07, 1.06))
        self.assertNotEqual(first["session"], second["session"])
        self.assertEqual(second["time_map_session"], "imu-boot-2")

    def test_integrated_clock_sync_configuration(self):
        args = bridge.parse_args(["--mode", "control", "--clock-endpoint", "tcp://192.168.60.153:5561"])
        self.assertTrue(args.time_map.endswith("/g1_clock_map.json"))
        args = bridge.parse_args(["--mode", "control", "--clock-endpoint", "tcp://127.0.0.1:5561",
                                  "--time-map", str(self.map_path)])
        self.assertEqual(args.time_map, str(self.map_path))
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            bridge.parse_args(["--clock-endpoint", "tcp://127.0.0.1:5561"])

    def test_control_configuration_requires_explicit_map(self):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            bridge.parse_args(["--mode", "control"])


if __name__ == "__main__":
    unittest.main()
