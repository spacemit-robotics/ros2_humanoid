#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Estimate ROS/system-to-HI91 time from read-only driver clock anchors.

No ROS dependency. The receive-delay budget comes from the driver configuration;
network measurements cannot establish the IMU's absolute one-way transport delay.
"""

import argparse
from collections import deque
import fcntl
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
import uuid

import numpy as np
import zmq


def default_map_path():
    return str(Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")) / "g1_clock_map.json")


class ClockEstimator:
    """Fit centered clocks; reject stale anchors, clock resets and excess uncertainty."""

    def __init__(self, max_age=2.0, max_error=0.02, min_span=5.0):
        if (not 0 < max_age <= 5 or not 0 < max_error <= .02 or min_span <= 0
                or not all(math.isfinite(v) for v in (max_age, max_error, min_span))):
            raise ValueError("invalid clock estimator limits")
        self.max_age, self.max_error, self.min_span = max_age, max_error, min_span
        self.samples = deque(maxlen=600)
        self.peer_session = None
        self.generation = uuid.uuid4().hex
        self.last_ros_pair = self.last_anchor = None

    def reset(self):
        self.samples.clear()
        self.last_anchor = None
        self.generation = uuid.uuid4().hex

    def observe(self, reply, t1, t4, ros_now, mono_now, pair_error=0.0):
        if (not isinstance(reply, dict) or reply.get("version") != 1 or reply.get("valid") is not True
                or reply.get("server_clock") != "monotonic"
                or not isinstance(reply.get("session"), str) or not reply["session"]):
            self.reset()
            raise ValueError("driver has no fresh clock anchor")
        names = ("t2_mono_s", "t3_mono_s", "anchor_robot_s", "anchor_receive_mono_s", "receive_delay_bound_s")
        for name in names:
            value = reply.get(name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                self.reset()
                raise ValueError(f"invalid anchor field: {name}")
        t2, t3 = reply["t2_mono_s"], reply["t3_mono_s"]
        receive, robot, bound = reply["anchor_receive_mono_s"], reply["anchor_robot_s"], reply["receive_delay_bound_s"]
        rtt = (t4 - t1) - (t3 - t2)
        if (t4 < t1 or t3 < t2 or not -1e-6 <= rtt <= 0.01 or t4 - t1 > 0.05
                or robot < 0 or not 0 < bound <= 0.02 or not 0 <= t2 - receive <= 0.2):
            raise ValueError("clock RTT, anchor age or receive-delay budget exceeds limit")
        # system/ROS time may slew under NTP; fit its rate as well as the device rate.
        if self.last_ros_pair is not None:
            previous_ros, previous_mono = self.last_ros_pair
            if abs((ros_now - previous_ros) - (mono_now - previous_mono)) > .01:
                self.reset()
        self.last_ros_pair = (ros_now, mono_now)
        if reply["session"] != self.peer_session:
            self.reset()
            self.peer_session = reply["session"]
        if self.last_anchor is not None:
            old_receive, old_robot = self.last_anchor
            if receive < old_receive or robot < old_robot:
                self.reset()
            elif receive == old_receive:
                raise ValueError("driver repeated its previous clock anchor")
        self.last_anchor = (receive, robot)
        # NTP four-timestamp offset: little-board monotonic minus brain monotonic.
        theta = ((t2 - t1) + (t3 - t4)) / 2
        # Unknown sensor->receive delay lies in [0, bound]; use midpoint and budget half.
        source_mono = receive - theta - bound / 2
        self.samples.append((source_mono, robot, mono_now, ros_now, max(rtt, 0) / 2 + pair_error, bound / 2))
        while self.samples and mono_now - self.samples[0][2] > 30:
            self.samples.popleft()
        values = np.asarray(self.samples)
        if len(values) < 30 or values[-1, 0] - values[0, 0] < self.min_span:
            return None
        x, y, host_mono, host_ros = values[:, 0], values[:, 1], values[:, 2], values[:, 3]

        def fit(a, b):
            center, mean = a.mean(), b.mean()
            delta = a - center
            slope = float(np.dot(delta, b - mean) / np.dot(delta, delta))
            residual = b - (mean + slope * delta)
            return slope, center, mean, float(np.max(np.abs(residual)))

        device_rate, center, mean, fit_error = fit(x, y)
        ros_rate, _, _, wall_error = fit(host_mono, host_ros)
        scale = device_rate / ros_rate
        target = mean + device_rate * (mono_now - center)
        # Prediction budget covers the full map TTL, not only this reply instant.
        error = float(values[:, 4].max() + values[:, 5].max() + fit_error + wall_error
                      + .001 + .002 * self.max_age)
        if not 0.999 <= scale <= 1.001 or not math.isfinite(target) or error > self.max_error:
            self.reset()
            raise ValueError(f"clock fit outside limits: scale={scale:.9f}, error={error * 1000:.2f} ms")
        return dict(version=1, source_clock="ros", target_clock="robot",
                    session=self.peer_session + "/" + self.generation,
                    source_time_s=ros_now, target_time_s=target, scale=scale,
                    updated_ros_time_s=ros_now, error_bound_s=error,
                    estimator_samples=len(values), network_rtt_s=max(rtt, 0),
                    receive_delay_bound_s=bound, fit_residual_s=fit_error)


class ClockSync:
    """Background-capable map writer; no interaction with robot control interfaces."""

    def __init__(self, endpoint, output, max_age=2.0, max_error=.02):
        self.endpoint, self.output = endpoint, Path(output)
        self.estimator = ClockEstimator(max_age=max_age, max_error=max_error)
        self.stop_event = threading.Event()
        self.thread = None
        self.output.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.lock = self.output.with_suffix(self.output.suffix + ".lock").open("a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock.close()
            raise ValueError(f"another clock synchronizer owns {self.output}") from None
        self.output.unlink(missing_ok=True)

    def invalidate(self):
        self.output.unlink(missing_ok=True)

    def write(self, mapping):
        descriptor, path = tempfile.mkstemp(prefix=self.output.name + ".", dir=self.output.parent)
        try:
            with os.fdopen(descriptor, "w") as stream:
                json.dump(mapping, stream, separators=(",", ":"), allow_nan=False)
                stream.write("\n")
            os.replace(path, self.output)
        finally:
            Path(path).unlink(missing_ok=True)

    def run(self):
        context, socket = zmq.Context(), None
        last_good, last_log = -math.inf, -math.inf
        try:
            while not self.stop_event.is_set():
                begin = time.monotonic()
                try:
                    if socket is None:
                        socket = context.socket(zmq.REQ)
                        socket.setsockopt(zmq.LINGER, 0)
                        socket.setsockopt(zmq.RCVTIMEO, 100)
                        socket.setsockopt(zmq.SNDTIMEO, 100)
                        socket.setsockopt(zmq.MAXMSGSIZE, 4096)
                        socket.connect(self.endpoint)
                    t1 = time.monotonic()
                    socket.send_json(dict(version=1, op="clock"))
                    data = socket.recv()
                    t4 = time.monotonic()
                    m0, ros_now = time.monotonic(), time.time()
                    m1 = time.monotonic()
                    mapping = self.estimator.observe(json.loads(data), t1, t4, ros_now, (m0 + m1) / 2, (m1 - m0) / 2)
                    if mapping is not None:
                        self.write(mapping)
                        last_good = time.monotonic()
                        if last_good - last_log >= 5:
                            print(f"[clock] ready: scale={mapping['scale']:.9f}, "
                                  f"error<={mapping['error_bound_s'] * 1000:.2f} ms, "
                                  f"RTT={mapping['network_rtt_s'] * 1000:.2f} ms", flush=True)
                            last_log = last_good
                    else:
                        self.invalidate()
                except zmq.ZMQError:
                    if socket is not None:
                        socket.close()
                        socket = None
                except (ValueError, TypeError, KeyError, OverflowError) as error:
                    self.invalidate()
                    if time.monotonic() - last_log >= 2:
                        print(f"[clock] waiting: {error}", flush=True)
                        last_log = time.monotonic()
                if time.monotonic() - last_good > .5:
                    self.invalidate()
                self.stop_event.wait(max(0, .1 - (time.monotonic() - begin)))
        finally:
            self.invalidate()
            if socket is not None:
                socket.close()
            context.term()
            self.lock.close()

    def start(self):
        self.thread = threading.Thread(target=self.run, name="g1_clock_sync", daemon=True)
        self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2)
        else:
            self.invalidate()
            self.lock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="tcp://192.168.60.153:5561")
    parser.add_argument("--output", default=default_map_path())
    parser.add_argument("--duration", type=float, default=0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("duration must be nonnegative")
    sync = ClockSync(args.endpoint, args.output)
    sync.start()
    try:
        start = time.monotonic()
        while not args.duration or time.monotonic() - start < args.duration:
            time.sleep(.1)
    except KeyboardInterrupt:
        pass
    finally:
        sync.close()


if __name__ == "__main__":
    main()
