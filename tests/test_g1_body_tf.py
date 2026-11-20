# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Exercise the hardware TF process with isolated operator WebSocket feedback."""

import asyncio
import copy
import json
import math
import os
import signal
import subprocess
import time
import threading
import xml.etree.ElementTree as ET

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from aiohttp import web, WSMsgType


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

    def __init__(self, tmp_path, publish_footprint, connection):
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
            os.environ['G1_BODY_TF_EXECUTABLE'], '--ros-args',
            '-p', 'base_height:=0.72', '-p', 'state_timeout_s:=0.15',
            '-p', f'publish_footprint_tf:={str(publish_footprint).lower()}',
            '-p', f'connection_file:={connection}',
        ], stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.01)
            time.sleep(.005)
        assert self.process.poll() is None

    def close(self):
        self.process.send_signal(signal.SIGINT)
        self.process.wait(timeout=5)
        self.log.close()
        self.listener.unregister()
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


class OperatorServer:
    """Local operator wire peer for the real C++ SDK; never talks to a robot."""

    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.advance = True
        self.operations = []
        self.subscriptions = 0
        self.clients = set()
        self.sample = dict(
            available=True, fresh=True, stream_id=1, sequence=1,
            source_time_s=10.0, received_at_unix_s=time.time(),
            age_ms=50.0, stale_after_ms=300.0, frame_id='base_link',
            joint_names=['waist_roll_joint', 'unused', 'waist_pitch_joint', 'waist_yaw_joint'],
            joint_positions=[-.17, 0, .24, .31], joint_velocities=[0]*4, joint_efforts=[],
            orientation=[2*v for v in quaternion(.15, -.23, 1.2)],
            angular_velocity=[0, 0, 0], acceleration=None)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        assert self.ready.wait(5)

    def update(self, advance=None, **values):
        with self.lock:
            self.sample.update(values)
            if advance is not None:
                self.advance = advance

    async def publish(self, socket):
        while not socket.closed:
            with self.lock:
                if self.advance:
                    self.sample['sequence'] += 1
                # Keep the device clock constant and offset driver receipt time:
                # the TF node must stamp with its own local ROS clock.
                self.sample['received_at_unix_s'] = time.time() - .05
                sample = copy.deepcopy(self.sample)
            await socket.send_json(dict(v=1, event='robot_feedback', data=sample))
            await asyncio.sleep(.02)

    async def handle(self, request):
        socket = web.WebSocketResponse()
        await socket.prepare(request)
        self.clients.add(socket)
        publisher = None
        try:
            async for message in socket:
                if message.type != WSMsgType.TEXT:
                    continue
                command = json.loads(message.data)
                op = command['op']
                self.operations.append(op)
                response = dict(v=1, id=command['id'], ok=True, code='ok', message='', data={})
                if op == 'hello':
                    assert command['args']['token'] == 'test-read-only'
                    response['data'] = dict(catalog=dict(robot='g1', policies=[], actions=[]), status=dict(
                        online=True, hmi_connected=True, zero_ready=True, state='RL', policy='walk',
                        age_ms=0, rl_hz=50, velocity=dict(vx=0, vy=0, wz=0), owner='another-client',
                        owns_control=False, lease_remaining_ms=0,
                        fault=dict(active=False, latched=False, sequence=0, source='', code='', detail=''),
                        interaction=dict(phase='IDLE', action='', progress=0),
                        request=dict(sequence=0, operation='', phase='', message='')))
                elif op != 'subscribe_robot_feedback':
                    response.update(ok=False, code='unexpected_control_request')
                await socket.send_json(response)
                if op == 'subscribe_robot_feedback':
                    self.subscriptions += 1
                    publisher = asyncio.create_task(self.publish(socket))
        finally:
            if publisher:
                publisher.cancel()
                await asyncio.gather(publisher, return_exceptions=True)
            self.clients.discard(socket)
        return socket

    def run(self):
        asyncio.set_event_loop(self.loop)
        app = web.Application()
        app.router.add_get('/api/v1/ws', self.handle)
        self.runner = web.AppRunner(app)
        self.loop.run_until_complete(self.runner.setup())
        site = web.TCPSite(self.runner, '127.0.0.1', 0)
        self.loop.run_until_complete(site.start())
        self.port = site._server.sockets[0].getsockname()[1]
        self.ready.set()
        self.loop.run_forever()
        self.loop.run_until_complete(self.runner.cleanup())
        self.loop.close()

    def disconnect(self):
        async def close_clients():
            for socket in list(self.clients):
                await socket.close()
        asyncio.run_coroutine_threadsafe(close_clients(), self.loop).result(timeout=5)

    def close(self):
        self.disconnect()
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=5)
        assert not self.thread.is_alive()


@pytest.fixture(params=[True, False], ids=['remote-imu-footprint', 'remote-external-footprint'])
def remote_runtime(request, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(140 + os.getpid() % 50))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    server = OperatorServer()
    connection = tmp_path / 'connection.json'
    connection.write_text(json.dumps(dict(
        endpoint=f'ws://127.0.0.1:{server.port}', token='test-read-only')))
    result = Runtime(tmp_path, request.param, connection)
    try:
        yield result, server, request.param
    finally:
        result.close()
        server.close()


def test_remote_geometry_local_stamp_and_reconnect(remote_runtime):
    rt, server, footprint = remote_runtime
    begin = time.time()
    rt.spin(1.5)
    assert rt.messages
    parent = 'base_footprint' if footprint else 'base_link'
    tf = rt.buffer.lookup_transform(parent, 'camera_link', rclpy.time.Time())
    angles = {'waist_yaw_joint': .31, 'waist_roll_joint': -.17, 'waist_pitch_joint': .24}
    want = expected(angles, (.15, -.23) if footprint else (0, 0), .72 if footprint else 0)
    for row, target in zip(transform_matrix(tf.transform), want):
        assert row == pytest.approx(target, abs=1e-9)
    assert begin < tf.header.stamp.sec + tf.header.stamp.nanosec/1e9 <= time.time()
    for message in rt.messages:
        assert len({(t.header.stamp.sec, t.header.stamp.nanosec) for t in message.transforms}) == 1
    if not footprint:
        assert not rt.buffer.can_transform('base_footprint', 'base_link', rclpy.time.Time())

    # Repeated samples with fresh transport timestamps must never be re-stamped.
    server.update(advance=False)
    rt.spin(.25)
    count = len(rt.messages)
    rt.spin(.25)
    assert len(rt.messages) == count
    # Reject lower sequence numbers within a stream, then accept a driver restart.
    server.update(sequence=1)
    rt.spin(.25)
    assert len(rt.messages) == count
    server.update(stream_id=2, sequence=1)
    rt.spin(.25)
    assert len(rt.messages) == count + 1
    server.update(advance=True)
    rt.spin(.25)
    assert len(rt.messages) > count + 1

    # The service can be reachable while reporting stale or incomplete feedback.
    for invalid in [dict(fresh=False), dict(available=False), dict(age_ms=1000),
                    dict(joint_positions=[]), dict(joint_names=['duplicate']*4)]:
        with server.lock:
            original = copy.deepcopy(server.sample)
        server.update(**invalid)
        rt.spin(.25)
        count = len(rt.messages)
        rt.spin(.2)
        assert len(rt.messages) == count
        server.update(**{key: original[key] for key in invalid})
        rt.spin(.2)
        assert len(rt.messages) > count

    # Missing IMU suppresses tilt, but an external footprint needs only waist data.
    server.update(orientation=None)
    rt.spin(.25)
    count = len(rt.messages)
    rt.spin(.2)
    assert (len(rt.messages) == count) == footprint
    server.update(orientation=quaternion(.15, -.23, 1.2))

    subscriptions = server.subscriptions
    server.disconnect()
    rt.spin(1.5)
    assert server.subscriptions > subscriptions
    assert len(rt.messages) > count
    assert set(server.operations) == {'hello', 'subscribe_robot_feedback'}
