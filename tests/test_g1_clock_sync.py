#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Clock fit, uncertainty and file ownership tests without ROS or robot hardware."""

import importlib.util
import json
from pathlib import Path
import tempfile
import time
from unittest.mock import patch

import zmq
import unittest

SPEC = importlib.util.spec_from_file_location(
    "g1_clock_sync", Path(__file__).resolve().parents[1] / "scripts/g1_clock_sync.py")
clock = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(clock)


def observation(index, robot_rate=1.00015, ros_rate=1.0003, session="robot-1"):
    t1 = 1000 + index * .1
    t4 = t1 + .0006
    mono = t4 + .0001
    ros = 1795000000 + ros_rate * (mono - 1000)
    sample = t1 - .02
    reply = dict(version=1, valid=True, server_clock="monotonic", session=session,
                 t2_mono_s=t1 + .0002 + 10000, t3_mono_s=t1 + .0003 + 10000,
                 anchor_robot_s=50 + robot_rate * (sample - 1000),
                 anchor_receive_mono_s=sample + .005 + 10000, receive_delay_bound_s=.01)
    return reply, t1, t4, ros, mono


class ClockSyncTest(unittest.TestCase):
    def filled(self):
        estimator = clock.ClockEstimator()
        mapping = None
        for i in range(80):
            mapping = estimator.observe(*observation(i))
        return estimator, mapping

    def test_maps_both_monotonic_and_ntp_slewing_ros_clocks(self):
        estimator, mapping = self.filled()
        self.assertIsNotNone(mapping)
        self.assertAlmostEqual(mapping["scale"], 1.00015 / 1.0003, places=6)
        expected = 50 + 1.00015 * (observation(79)[4] - 1000)
        self.assertLess(abs(mapping["target_time_s"] - expected), .001)
        self.assertLess(mapping["error_bound_s"], .02)
        self.assertGreater(mapping["error_bound_s"], .009)
        self.assertEqual(mapping["source_time_s"], observation(79)[3])

    def test_warmup_and_robot_restart(self):
        estimator, mapping = self.filled()
        generation = mapping["session"]
        self.assertIsNone(estimator.observe(*observation(80, session="robot-2")))
        self.assertEqual(len(estimator.samples), 1)
        for i in range(81, 150):
            new = estimator.observe(*observation(i, session="robot-2"))
        self.assertNotEqual(new["session"], generation)

    def test_future_stale_and_invalid_response(self):
        for changes in [dict(valid=False), dict(t2_mono_s=10120), dict(t3_mono_s=9990),
                        dict(anchor_receive_mono_s=9900), dict(anchor_robot_s=float("nan")),
                        dict(receive_delay_bound_s=.1), dict(server_clock="wall"), dict(session="")]:
            with self.subTest(changes=changes):
                args = list(observation(1))
                args[0].update(changes)
                with self.assertRaises(ValueError):
                    clock.ClockEstimator().observe(*args)

    def test_duplicate_anchor_not_used_as_new_measurement(self):
        estimator = clock.ClockEstimator()
        args = observation(0)
        estimator.observe(*args)
        with self.assertRaisesRegex(ValueError, "repeated"):
            estimator.observe(*args)
        self.assertEqual(len(estimator.samples), 1)

    def test_ros_step_discards_previous_fit(self):
        estimator, _ = self.filled()
        args = list(observation(80))
        args[3] += 1
        self.assertIsNone(estimator.observe(*args))
        self.assertEqual(len(estimator.samples), 1)

    def test_excessive_robot_drift_cannot_make_control_map(self):
        estimator = clock.ClockEstimator()
        with self.assertRaisesRegex(ValueError, "outside limits"):
            for i in range(100):
                estimator.observe(*observation(i, robot_rate=1.01))

    def test_transport_outage_removes_previously_valid_map(self):
        _, mapping = self.filled()
        context = zmq.Context()
        server = context.socket(zmq.REP)
        server.setsockopt(zmq.LINGER, 0)
        server.setsockopt(zmq.RCVTIMEO, 1000)
        port = server.bind_to_random_port("tcp://127.0.0.1")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "map.json"
            sync = clock.ClockSync(f"tcp://127.0.0.1:{port}", output)
            try:
                # Exercise the real REQ transport and writer; fit math is tested above.
                with patch.object(sync.estimator, "observe", return_value=mapping):
                    sync.start()
                    server.recv()
                    server.send_json({"version": 1})
                    deadline = time.monotonic() + 1
                    while not output.exists() and time.monotonic() < deadline:
                        time.sleep(.01)
                    self.assertTrue(output.exists())
                    server.close()
                    deadline = time.monotonic() + 1.5
                    while output.exists() and time.monotonic() < deadline:
                        time.sleep(.01)
                    self.assertFalse(output.exists())
            finally:
                sync.close()
                server.close()
                context.term()

    def test_atomic_map_writer_exclusive_ownership_and_cleanup(self):
        _, mapping = self.filled()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "map.json"
            sync = clock.ClockSync("tcp://127.0.0.1:1", output)
            try:
                sync.write(mapping)
                self.assertEqual(json.loads(output.read_text()), mapping)
                self.assertEqual(output.stat().st_mode & 0o777, 0o600)
                with self.assertRaisesRegex(ValueError, "another"):
                    clock.ClockSync("tcp://127.0.0.1:1", output)
                self.assertTrue(output.exists())
            finally:
                sync.close()
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
