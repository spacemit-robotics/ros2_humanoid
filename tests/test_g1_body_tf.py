# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Exercise the hardware TF process with isolated native-protocol SHM feedback."""

import ctypes as ct
import math
import mmap
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import xml.etree.ElementTree as ET

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


# Packed v7 wire layout from humanoid_common/transport_packet.h, not a ROS mock.
class Header(ct.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [('magic', ct.c_uint32), ('version', ct.c_uint16),
                ('type', ct.c_uint16), ('seq', ct.c_uint32)]


class Fault(ct.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [('seq', ct.c_uint64), ('time', ct.c_double), ('native', ct.c_int32),
                ('code', ct.c_int16), ('source', ct.c_uint8), ('active', ct.c_uint8),
                ('latched', ct.c_uint8), ('detail', ct.c_char * 160)]


class Packet(ct.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [('header', Header), ('num_dof', ct.c_int32), ('time', ct.c_double)] + [
        (name, ct.c_double * 3) for name in ('rpy', 'gyro', 'acceleration', 'base_pos')
    ] + [('base_quat', ct.c_double * 4), ('base_vel', ct.c_double * 6)] + [
        (name, ct.c_double * 64) for name in
        ('joint_pos', 'joint_vel', 'joint_torque', 'joint_temperature')
    ] + [('joint_error', ct.c_uint32 * 64), ('fault', Fault)]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def matrix(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y))
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]],
            [-sp, cp*sr, cp*cr, xyz[2]], [0, 0, 0, 1]]


def quaternion(r, p, y):
    cr, sr, cp, sp, cy, sy = (
        math.cos(r/2), math.sin(r/2), math.cos(p/2),
        math.sin(p/2), math.cos(y/2), math.sin(y/2))
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]


def expected(angles, tilt=(0, 0), height=0.72):
    model = ET.parse(os.environ['G1_TF_MODEL']).getroot()
    result = matrix((0, 0, height), (*tilt, 0))
    joints = ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint', 'd435_joint')
    for name in joints:
        joint = model.find(f"joint[@name='{name}']")
        origin = joint.find('origin')
        xyz = tuple(map(float, origin.get('xyz').split()))
        rpy = tuple(map(float, origin.get('rpy').split()))
        result = multiply(result, matrix(xyz, rpy))
        if name in angles:
            axis = tuple(map(float, joint.find('axis').get('xyz').split()))
            result = multiply(result, matrix(rpy=tuple(a*angles[name] for a in axis)))
    return result


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    return [[1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.w*q.z),
             2*(q.x*q.z+q.w*q.y), t.x],
            [2*(q.x*q.y+q.w*q.z), 1-2*(q.x*q.x+q.z*q.z),
             2*(q.y*q.z-q.w*q.x), t.y],
            [2*(q.x*q.z-q.w*q.y), 2*(q.y*q.z+q.w*q.x),
             1-2*(q.x*q.x+q.y*q.y), t.z], [0, 0, 0, 1]]


class Runtime:

    def __init__(self, tmp_path, publish_footprint):
        self.prefix = f'g1_tf_test_{os.getpid()}'
        self.path = Path('/dev/shm') / f'hmrs_{self.prefix}_state'
        self.open_channel()
        self.config = tmp_path / 'robot.yaml'
        # Deliberately reorder the waist joints to check name-based indexing.
        self.config.write_text(
            'robot_base:\n  joint_names: [waist_roll_joint, unused1, '
            'waist_pitch_joint, unused2, waist_yaw_joint]\n'
            f'transport:\n  type: shm\n  shm:\n    prefix: {self.prefix}\n')
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('g1_tf_test', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.buffer = Buffer(node=self.node)
        self.listener = TransformListener(self.buffer, self.node)
        self.messages = []
        self.subscription = self.node.create_subscription(
            TFMessage, '/tf', self.messages.append, 100)
        self.log = (tmp_path / 'node.log').open('w+')
        self.process = subprocess.Popen([
            os.environ['G1_BODY_TF_EXECUTABLE'], str(self.config), '--ros-args',
            '-p', 'base_height:=0.72', '-p', 'state_timeout_s:=0.15',
            '-p', f'publish_footprint_tf:={str(publish_footprint).lower()}',
        ], stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.packet = Packet()
        self.packet.header = Header(0x484D5253, 7, 2, 0)
        self.packet.num_dof = 5
        self.packet.base_quat[0] = 1
        self.packet.time = 10
        self.seq = 0

    def open_channel(self):
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_EXCL, 0o600)
        try:
            os.ftruncate(fd, 64 + 4*4096)
            self.shm = mmap.mmap(fd, 64 + 4*4096)
            struct.pack_into('<IIII', self.shm, 0, 0, 2, 4, 4096)
        finally:
            os.close(fd)

    def pump(self, seconds, advance=True, write=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if write:
                self.seq += 1
                self.packet.header.seq = self.seq
                if advance:
                    self.packet.time += .01
                index = struct.unpack_from('<I', self.shm, 0)[0]
                start = 64 + index*4096
                self.shm[start:start+ct.sizeof(self.packet)] = bytes(self.packet)
                struct.pack_into('<I', self.shm, 0, (index+1) % 4)
            self.executor.spin_once(timeout_sec=.01)
            time.sleep(.005)
        assert self.process.poll() is None
        # Observation must never consume control_runtime's read cursor.
        assert struct.unpack_from('<I', self.shm, 4)[0] == 2

    def close(self):
        self.process.send_signal(signal.SIGINT)
        self.process.wait(timeout=5)
        self.log.close()
        self.listener.unregister()
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self.shm.close()
        self.path.unlink()


@pytest.fixture(params=[True, False], ids=['imu-footprint', 'external-footprint'])
def runtime(request, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(140 + os.getpid() % 50))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    result = Runtime(tmp_path, request.param)
    try:
        yield result, request.param
    finally:
        result.close()


def test_feedback_geometry_and_dropout(runtime):
    rt, footprint = runtime
    # New sequence numbers with a frozen device clock must not produce TF.
    rt.pump(.7, advance=False)
    assert not rt.messages
    rt.packet.joint_pos[4] = .31
    rt.packet.joint_pos[0] = -.17
    rt.packet.joint_pos[2] = .24
    # Scale the quaternion to check normalization; yaw must be removed from tilt TF.
    rt.packet.base_quat[:] = [2*v for v in quaternion(.15, -.23, 1.2)]
    rt.pump(.7)
    assert rt.messages
    parent = 'base_footprint' if footprint else 'base_link'
    tf = rt.buffer.lookup_transform(parent, 'camera_link', rclpy.time.Time())
    angles = {'waist_yaw_joint': .31, 'waist_roll_joint': -.17, 'waist_pitch_joint': .24}
    want = expected(angles, (.15, -.23) if footprint else (0, 0), .72 if footprint else 0)
    actual = transform_matrix(tf.transform)
    for row, target in zip(actual, want):
        assert row == pytest.approx(target, abs=1e-9)
    if not footprint:
        assert not rt.buffer.can_transform('base_footprint', 'base_link', rclpy.time.Time())
    for msg in rt.messages:
        assert len({(t.header.stamp.sec, t.header.stamp.nanosec) for t in msg.transforms}) == 1
    assert 'mid360' not in rt.buffer.all_frames_as_yaml()
    # A second feedback pose must change the computed camera transform.
    rt.packet.joint_pos[4] = -.2
    rt.pump(.3)
    changed = rt.buffer.lookup_transform(parent, 'camera_link', rclpy.time.Time())
    assert changed.transform.translation != tf.transform.translation
    # Sequence progress alone is insufficient when the device timestamp is frozen.
    rt.pump(.3, advance=False)
    count = len(rt.messages)
    rt.pump(.25, advance=False)
    assert len(rt.messages) == count
    # Invalid feedback must not generate new transforms either.
    rt.packet.joint_pos[4] = float('nan')
    rt.pump(.3)
    assert len(rt.messages) == count
    rt.packet.joint_pos[4] = 0
    rt.pump(.3)
    assert len(rt.messages) > count
    # Active native feedback faults suppress TF even with advancing samples.
    rt.packet.fault = Fault(1, rt.packet.time, 0, 2, 2, 1, 1, b'feedback timeout')
    rt.pump(.25)
    count = len(rt.messages)
    rt.pump(.25)
    assert len(rt.messages) == count
    rt.packet.fault = Fault()
    rt.pump(.3)
    assert len(rt.messages) > count
    # Driver restart recreates the SHM object; the observer reconnects after timeout.
    rt.shm.close()
    rt.path.unlink()
    rt.open_channel()
    rt.pump(.25, write=False)
    count = len(rt.messages)
    rt.pump(.25, write=False)
    assert len(rt.messages) == count
    rt.pump(.4)
    assert len(rt.messages) > count
