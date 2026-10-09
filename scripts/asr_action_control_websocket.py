#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0

"""Recognize LingLong actions and control hmi_runtime over WebSocket."""

import argparse
import base64
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import struct
import time
from urllib.parse import urlsplit

import asr_action_control as legacy


BUSY_INTERACTION_PHASES = {"BLEND_IN", "PLAYING", "HOLDING", "BLEND_OUT"}
TERMINAL_INTERACTION_PHASES = {"FINISHED", "REJECTED"}
TERMINAL_REQUEST_PHASES = {"completed", "rejected", "expired", "cancelled"}


class OperatorError(RuntimeError):
    """An hmi_runtime API request failed."""

    def __init__(self, code, message):
        super().__init__(f"{code}: {message}" if code else message)
        self.code = code
        self.message = message


class WebSocket:
    """Small RFC 6455 client for the local, unencrypted operator endpoint."""

    def __init__(self, endpoint, timeout_ms):
        parsed = urlsplit(endpoint)
        if parsed.scheme != "ws" or not parsed.hostname:
            raise ValueError("connection endpoint must use ws://host[:port]")
        self.host = parsed.hostname
        self.port = parsed.port or 80
        self.path = parsed.path if parsed.path not in ("", "/") else "/api/v1/ws"
        if parsed.query:
            self.path += "?" + parsed.query
        self.timeout_s = timeout_ms / 1000.0
        self.socket = None
        self.buffer = bytearray()
        self.fragments = bytearray()
        self.fragment_opcode = None

    def connect(self):
        self.socket = socket.create_connection(
            (self.host, self.port), timeout=self.timeout_s)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        request = (
            f"GET {self.path} HTTP/1.1\r\n"
            f"Host: {self.host}:{self.port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        self.socket.sendall(request.encode("ascii"))
        header = self._read_until(b"\r\n\r\n", 65536)
        lines = header.decode("iso-8859-1").split("\r\n")
        if not lines or " 101 " not in f" {lines[0]} ":
            raise RuntimeError(f"WebSocket upgrade failed: {lines[0] if lines else ''}")
        headers = {}
        for line in lines[1:]:
            if ":" in line:
                name, value = line.split(":", 1)
                headers[name.strip().lower()] = value.strip()
        expected = base64.b64encode(hashlib.sha1(
            (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11")
            .encode("ascii")).digest()).decode("ascii")
        if headers.get("sec-websocket-accept") != expected:
            raise RuntimeError("WebSocket server returned an invalid accept key")

    def _read_until(self, marker, limit):
        while marker not in self.buffer:
            chunk = self.socket.recv(4096)
            if not chunk:
                raise ConnectionError("WebSocket closed during handshake")
            self.buffer.extend(chunk)
            if len(self.buffer) > limit:
                raise RuntimeError("WebSocket handshake response is too large")
        end = self.buffer.index(marker) + len(marker)
        result = bytes(self.buffer[:end])
        del self.buffer[:end]
        return result

    def _read_exact(self, size):
        while len(self.buffer) < size:
            chunk = self.socket.recv(max(4096, size - len(self.buffer)))
            if not chunk:
                raise ConnectionError("WebSocket connection closed")
            self.buffer.extend(chunk)
        result = bytes(self.buffer[:size])
        del self.buffer[:size]
        return result

    def _send_frame(self, opcode, payload=b""):
        if self.socket is None:
            raise ConnectionError("WebSocket is not connected")
        payload = bytes(payload)
        mask = os.urandom(4)
        length = len(payload)
        header = bytearray([0x80 | opcode])
        if length < 126:
            header.append(0x80 | length)
        elif length <= 0xFFFF:
            header.append(0x80 | 126)
            header.extend(struct.pack("!H", length))
        else:
            header.append(0x80 | 127)
            header.extend(struct.pack("!Q", length))
        masked = bytes(value ^ mask[index % 4]
                       for index, value in enumerate(payload))
        self.socket.sendall(bytes(header) + mask + masked)

    def send_json(self, value):
        data = json.dumps(value, separators=(",", ":"),
                          ensure_ascii=False).encode("utf-8")
        self._send_frame(0x1, data)

    def receive_json(self, timeout_s=None):
        if timeout_s is not None:
            self.socket.settimeout(timeout_s)
        while True:
            first, second = self._read_exact(2)
            final = bool(first & 0x80)
            opcode = first & 0x0F
            masked = bool(second & 0x80)
            length = second & 0x7F
            if length == 126:
                length = struct.unpack("!H", self._read_exact(2))[0]
            elif length == 127:
                length = struct.unpack("!Q", self._read_exact(8))[0]
            mask = self._read_exact(4) if masked else None
            payload = self._read_exact(length)
            if mask:
                payload = bytes(value ^ mask[index % 4]
                                for index, value in enumerate(payload))
            if opcode == 0x8:
                raise ConnectionError("WebSocket server closed the connection")
            if opcode == 0x9:
                self._send_frame(0xA, payload)
                continue
            if opcode == 0xA:
                continue
            if opcode in (0x1, 0x2):
                self.fragments = bytearray(payload)
                self.fragment_opcode = opcode
            elif opcode == 0x0 and self.fragment_opcode is not None:
                self.fragments.extend(payload)
            else:
                continue
            if not final:
                continue
            if self.fragment_opcode != 0x1:
                self.fragments.clear()
                self.fragment_opcode = None
                continue
            data = bytes(self.fragments)
            self.fragments.clear()
            self.fragment_opcode = None
            return json.loads(data.decode("utf-8"))

    def close(self):
        if self.socket is None:
            return
        try:
            self._send_frame(0x8, struct.pack("!H", 1000))
        except (OSError, ConnectionError):
            pass
        try:
            self.socket.close()
        finally:
            self.socket = None


class OperatorClient:
    """Authenticated hmi_runtime client with explicit lease ownership."""

    def __init__(self, connection_file, timeout_ms, name):
        connection = json.loads(Path(connection_file).expanduser().read_text())
        self.websocket = WebSocket(connection["endpoint"], timeout_ms)
        self.timeout_s = timeout_ms / 1000.0
        self.token = connection["token"]
        self.name = name
        self.request_id = 0
        self.status = None
        self.catalog = None
        self.owned = False
        self.lease_ms = 1000
        self.next_renew_at = float("inf")

    def connect(self):
        self.websocket.connect()
        reply = self.call("hello", {"token": self.token, "name": self.name})
        self.catalog = reply["data"]["catalog"]
        self.status = reply["data"]["status"]
        self.lease_ms = int(self.catalog.get("lease_ms", 1000))

    def call(self, operation, args=None):
        self.request_id += 1
        request_id = self.request_id
        self.websocket.send_json({
            "v": 1,
            "id": request_id,
            "op": operation,
            "args": args or {},
        })
        deadline = time.monotonic() + self.timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"WebSocket {operation} request timed out")
            message = self.websocket.receive_json(remaining)
            if message.get("event") == "status":
                self.status = message.get("data", self.status)
                continue
            if message.get("id") != request_id:
                continue
            if not message.get("ok"):
                raise OperatorError(message.get("code", "error"),
                                    message.get("message", "request failed"))
            return message

    def refresh_status(self):
        reply = self.call("status")
        self.status = reply["data"]["status"]
        return self.status

    def acquire(self):
        self.call("acquire")
        self.owned = True
        self.next_renew_at = time.monotonic() + self.lease_ms / 4000.0
        self.refresh_status()

    def maintain_lease(self):
        if self.owned and time.monotonic() >= self.next_renew_at:
            self.call("renew")
            self.next_renew_at = time.monotonic() + self.lease_ms / 4000.0

    def release(self):
        if not self.owned:
            return
        try:
            self.call("release")
        finally:
            self.owned = False
            self.next_renew_at = float("inf")

    def close(self):
        try:
            self.release()
        finally:
            self.websocket.close()


def request_phase(status):
    return status.get("request", {}).get("phase", "")


def enter_action_mode(client, timeout_s, poll_interval_s):
    """Advance the configured state chain without leaving ZERO prematurely."""
    deadline = time.monotonic() + timeout_s
    last_reported = None
    while True:
        client.maintain_lease()
        status = client.refresh_status()
        state = status.get("state", "")
        if state != last_reported:
            print(f"[MODE] {state}", flush=True)
            last_reported = state
        fault = status.get("fault", {})
        if fault.get("latched"):
            raise RuntimeError(
                f"Control fault is latched: {fault.get('code', '')} "
                f"{fault.get('detail', '')}".strip())
        if state == "TRAJECTORY":
            return status
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"timed out entering TRAJECTORY; current state={state}, "
                f"zero_ready={status.get('zero_ready')}")
        phase = request_phase(status)
        if phase == "accepted":
            time.sleep(poll_interval_s)
            continue
        target = None
        if state == "POWER_OFF":
            target = "DAMP"
        elif state == "DAMP":
            target = "HOME"
        elif state == "HOME":
            target = "ZERO"
        elif state == "ZERO":
            if status.get("zero_ready"):
                target = "TRAJECTORY"
        elif state == "SAFETY":
            raise RuntimeError("Control is in SAFETY; return to POWER_OFF first")
        else:
            target = "DAMP"
        if target is not None:
            print(f"[MODE] request {state} -> {target}", flush=True)
            client.call("state", {"state": target})
        time.sleep(poll_interval_s)


def wait_for_action(client, timeout_s, poll_interval_s):
    deadline = time.monotonic() + timeout_s
    last_phase = None
    while True:
        client.maintain_lease()
        status = client.refresh_status()
        phase = status.get("interaction", {}).get("phase", "IDLE")
        if phase != last_phase:
            print(f"[ACTION] {phase}", flush=True)
            last_phase = phase
        if phase in TERMINAL_INTERACTION_PHASES:
            if phase == "REJECTED":
                raise RuntimeError("Control rejected the interaction")
            return
        operation_phase = request_phase(status)
        if operation_phase in TERMINAL_REQUEST_PHASES and operation_phase != "completed":
            message = status.get("request", {}).get("message", "")
            raise RuntimeError(f"interaction {operation_phase}: {message}")
        if time.monotonic() >= deadline:
            try:
                client.call("cancel")
            except OperatorError:
                pass
            raise TimeoutError(f"interaction did not finish within {timeout_s:.1f}s")
        time.sleep(poll_interval_s)


def request_action(client, action, args):
    try:
        client.acquire()
    except OperatorError as error:
        if error.code == "busy":
            print("[SKIP] Web operator currently owns control")
            return False
        raise
    try:
        status = client.status
        if not status.get("online"):
            print("[SKIP] Control feedback is offline")
            return False
        if args.auto_enter_mode:
            status = enter_action_mode(
                client, args.mode_timeout_s, args.poll_interval_s)
        if status.get("state") != "TRAJECTORY":
            print(f"[SKIP] Control is not ready in TRAJECTORY: "
                  f"{status.get('state')}")
            return False
        phase = status.get("interaction", {}).get("phase", "IDLE")
        if phase in BUSY_INTERACTION_PHASES:
            print("[SKIP] an interaction is already playing")
            return False
        reply = client.call("interaction", {"action": action})
        print(f"[WS] queued {action}: {reply.get('message', '')}")
        wait_for_action(client, args.action_timeout_s, args.poll_interval_s)
        print(f"[WS] completed {action}")
        return True
    finally:
        client.release()


def build_parser():
    parser = argparse.ArgumentParser(
        description="LingLong standing action voice control over WebSocket")
    parser.add_argument("-d", "--device", type=int, default=-1)
    parser.add_argument("--playback-device", type=int, default=-1)
    parser.add_argument("--playback-rate", type=int, default=48000)
    parser.add_argument("--playback-channels", type=int, default=2)
    parser.add_argument("-r", "--rate", type=int, default=16000)
    parser.add_argument("-c", "--channels", type=int, default=2)
    parser.add_argument("-l", "--list-devices", action="store_true")
    parser.add_argument("--asr-backend", choices=("sensevoice", "qwen3_asr"),
                        default="sensevoice")
    parser.add_argument("--asr-threads", type=int, default=4)
    parser.add_argument("--qwen3-endpoint",
                        default="http://127.0.0.1:8063/v1/chat/completions")
    parser.add_argument("--qwen3-model", default="qwen3-asr")
    parser.add_argument("--qwen3-timeout-sec", type=float, default=10.0)
    parser.add_argument("--qwen3-model-dir",
                        default="~/.cache/models/asr/qwen3asr/qwen3-asr-0.6B-dynq-q40")
    parser.add_argument("--qwen3-server-threads", type=int, default=4)
    parser.add_argument("--qwen3-startup-timeout-sec", type=float, default=90.0)
    parser.add_argument("--qwen3-max-transcript-chars", type=int, default=32)
    parser.add_argument("--qwen3-no-auto-start", action="store_true")
    parser.add_argument(
        "--connection-file",
        default="~/.local/state/humanoid-operator/linglong/connection.json")
    parser.add_argument("--socket-timeout-ms", type=int, default=1500)
    parser.add_argument("--mode-timeout-s", type=float, default=20.0)
    parser.add_argument("--action-timeout-s", type=float, default=30.0)
    parser.add_argument("--poll-interval-s", type=float, default=0.1)
    parser.add_argument("--no-auto-enter-mode", dest="auto_enter_mode",
                        action="store_false",
                        help="require TRAJECTORY to be ready before an action")
    parser.set_defaults(auto_enter_mode=True)
    parser.add_argument("--match-threshold", type=float, default=0.72)
    parser.add_argument("--cooldown-s", type=float, default=3.0)
    parser.add_argument("--max-utterance-s", type=float, default=8.0)
    parser.add_argument("--wake-word", default="")
    parser.add_argument("--no-audio-prompts", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--text", help="match one sentence without a microphone")
    return parser


def validate_args(parser, args):
    if not 0.0 < args.match_threshold <= 1.0:
        parser.error("--match-threshold must be in (0, 1]")
    finite_positive = (
        args.max_utterance_s, args.mode_timeout_s,
        args.action_timeout_s, args.poll_interval_s)
    if (not math.isfinite(args.cooldown_s) or args.cooldown_s < 0 or
            any(not math.isfinite(value) or value <= 0
                for value in finite_positive) or
            args.socket_timeout_ms <= 0):
        parser.error("cooldown must be nonnegative; timeouts must be positive")
    if (args.asr_threads <= 0 or args.playback_rate <= 0 or
            args.playback_channels <= 0 or args.qwen3_timeout_sec <= 0 or
            args.qwen3_server_threads <= 0 or
            args.qwen3_startup_timeout_sec <= 0 or
            args.qwen3_max_transcript_chars <= 0):
        parser.error("audio format, thread counts and limits must be positive")
    if args.text is not None and args.list_devices:
        parser.error("--text and --list-devices cannot be combined")


def main():
    parser = build_parser()
    args = parser.parse_args()
    validate_args(parser, args)
    client = None
    if not args.dry_run and not args.list_devices:
        client = OperatorClient(
            args.connection_file, args.socket_timeout_ms,
            "asr_action_control_websocket")
        try:
            client.connect()
        except Exception as error:
            client.close()
            parser.exit(1, f"[ERROR] WebSocket connection failed: {error}\n")
    last_sent_at = [float("-inf")]

    def handle_text(text):
        matched = legacy.match_action(
            text, args.match_threshold, args.wake_word)
        if matched is None:
            print(f"[SKIP] no unambiguous action: {text}")
            if (not args.no_audio_prompts and not args.dry_run and
                    (not args.wake_word or
                     legacy.has_wake_word(text, args.wake_word))):
                return "unclear"
            return None
        action, score = matched
        print(f"[MATCH] {text} -> {action} ({score:.2f})")
        if args.dry_run:
            return None if args.no_audio_prompts else "executing"
        now = time.monotonic()
        if now - last_sent_at[0] < args.cooldown_s:
            print("[SKIP] action cooldown")
            return None
        if request_action(client, action, args):
            last_sent_at[0] = time.monotonic()
            return None if args.no_audio_prompts else "executing"
        return None

    try:
        if args.text is not None:
            try:
                prompt = handle_text(args.text)
                if prompt is not None:
                    legacy.play_prompt(
                        prompt, args.playback_device,
                        args.playback_rate, args.playback_channels)
            except Exception as error:
                parser.exit(1, f"[ERROR] {error}\n")
        else:
            legacy.run_audio(args, handle_text)
    finally:
        if client is not None:
            client.close()


if __name__ == "__main__":
    main()
