#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0

"""Recognize spoken LingLong standing actions and request them over ZMQ."""

import argparse
import collections
from difflib import SequenceMatcher
import math
import queue
import re
import signal
import threading
import time
import unicodedata


# Keys must be registered in linglong.yaml: upper_body_actions.action_names.
ACTION_PHRASES = {
    "wave_hello": ("挥手", "挥挥手", "招手", "打招呼", "你好"),
    "wave_under_head": ("胸前挥手", "胸口挥手", "低位挥手"),
    "wave_above_head": ("高举挥手", "头顶挥手", "举手挥手"),
    "heart_both": ("比心", "双手比心", "两手比心"),
    "heart_left": ("左手比心", "左边比心"),
    "heart_right": ("右手比心", "右边比心"),
    "shake_hand": ("握手", "右手握手", "伸手握手"),
    "shake_hand_left": ("左手握手", "左边握手"),
    "blow_kiss_with_left_hand": ("左手飞吻", "左边飞吻"),
    "blow_kiss_with_right_hand": ("右手飞吻", "右边飞吻"),
    "blow_kiss_with_both_hands": ("双手飞吻", "两手飞吻"),
    "hug": ("拥抱", "抱抱", "欢迎"),
    "high_five": ("击掌", "拍手", "和我击掌"),
    "right_hand_up": ("右手摊手", "右手举起", "举右手"),
    "both_hands_up": ("双手摊手", "双手举起", "举双手"),
    "box_left_hand_win": ("胜利庆祝", "庆祝胜利"),
    "right_hand_on_heart": ("抚胸致意", "右手放胸前"),
    "ultraman_ray": ("奥特曼光线", "发射光线"),
}

PREFIXES = ("小机器人", "机器人", "请你", "请", "帮我", "给我", "来一个",
            "来个", "做一个", "做个", "表演一个", "表演")
SUFFIXES = ("动作", "一下", "好吗", "好不好", "可以吗", "吧", "呀", "啊")
BUSY_PHASES = {"进入", "播放", "保持", "收回"}


def clean_text(text):
    """Remove punctuation and spacing from ASR text."""
    text = unicodedata.normalize("NFKC", text).lower()
    return re.sub(r"[^\w\u4e00-\u9fff]", "", text)


def normalize_text(text):
    """Remove punctuation and common command fillers from ASR text."""
    text = clean_text(text)
    for _ in range(3):
        for prefix in PREFIXES:
            if text.startswith(prefix):
                text = text[len(prefix):]
                break
        else:
            break
    for _ in range(3):
        for suffix in SUFFIXES:
            if text.endswith(suffix):
                text = text[:-len(suffix)]
                break
        else:
            break
    return text


def match_action(text, threshold=0.72, wake_word=""):
    """Return (action, score) only for an unambiguous registered phrase."""
    spoken = clean_text(text)
    if wake_word:
        wake = clean_text(wake_word)
        if not wake or wake not in spoken:
            return None
        spoken = spoken.replace(wake, "", 1)
    spoken = normalize_text(spoken)
    if not spoken:
        return None

    exact = []
    fuzzy = []
    for action, phrases in ACTION_PHRASES.items():
        normalized = [normalize_text(phrase) for phrase in phrases]
        contained = [phrase for phrase in normalized if phrase in spoken]
        if contained and len(spoken) <= 20:
            exact.append((max(map(len, contained)), action))
        score = max(SequenceMatcher(None, spoken, phrase).ratio()
                    for phrase in normalized)
        fuzzy.append((score, action))

    if exact:
        longest = max(length for length, _ in exact)
        winners = {action for length, action in exact if length == longest}
        if len(winners) == 1:
            return winners.pop(), 1.0
        return None

    fuzzy.sort(reverse=True)
    if (fuzzy[0][0] >= threshold and
            fuzzy[0][0] - fuzzy[1][0] >= 0.06):
        return fuzzy[0][1], fuzzy[0][0]
    return None


class HmiClient:
    """Use a fresh REQ socket for each request so timeouts cannot wedge it."""

    def __init__(self, endpoint, timeout_ms):
        import zmq

        self.zmq = zmq
        self.context = zmq.Context()
        self.endpoint = endpoint
        self.timeout_ms = timeout_ms

    def request(self, payload):
        socket = self.context.socket(self.zmq.REQ)
        socket.setsockopt(self.zmq.LINGER, 0)
        socket.setsockopt(self.zmq.SNDTIMEO, self.timeout_ms)
        socket.setsockopt(self.zmq.RCVTIMEO, self.timeout_ms)
        socket.connect(self.endpoint)
        try:
            socket.send_json(payload)
            return socket.recv_json()
        finally:
            socket.close()

    def close(self):
        self.context.term()


def request_action(client, action):
    """Check the real Control status before queuing a standing interaction."""
    status = client.request({"op": "status"})
    if not isinstance(status, dict) or not status.get("ok"):
        raise RuntimeError(f"HMI status request failed: {status}")
    if (not status.get("online") or status.get("fault") or
            status.get("mode") != "RL" or
            status.get("policy") != "stand_mjlab" or status.get("switching")):
        print("[SKIP] Control is not ready in stand_mjlab/RL")
        return False
    if status.get("interaction_phase") in BUSY_PHASES:
        print("[SKIP] an interaction is already playing")
        return False
    reply = client.request({"op": "interaction", "action": action})
    if not isinstance(reply, dict) or not reply.get("ok"):
        raise RuntimeError(f"HMI rejected {action}: {reply}")
    print(f"[ZMQ] queued {action}; check HMI status for execution result")
    return True


def run_audio(args, on_text):
    """Capture audio and run VAD/ASR as in asr_simple.py."""
    import numpy as np
    import spacemit_audio
    from spacemit_audio import AudioCapture
    import spacemit_asr
    import spacemit_vad

    if args.list_devices:
        for index, name in AudioCapture.list_devices():
            print(f"  [{index}] {name}")
        return

    vad_config = (spacemit_vad.VadConfig.preset("silero")
                  .with_trigger_threshold(0.4)
                  .with_stop_threshold(0.3)
                  .with_min_speech_duration(100)
                  .with_smoothing(False))
    vad = spacemit_vad.VadEngine(vad_config)
    asr_config = spacemit_asr.Config()
    asr_config.provider = "cpu"
    asr_config._config.num_threads = 4
    asr_config.language = spacemit_asr.Language.ZH
    asr_config.punctuation = True
    asr = spacemit_asr.Engine(asr_config).initialize()
    print(f"VAD: {vad.engine_name}; ASR: {asr.backend_name}")
    asr.recognize(np.zeros(16000, dtype=np.float32))

    target_rate = 16000
    resampler = (spacemit_asr.Resampler(args.rate, target_rate, channels=1)
                 if args.rate != target_rate else None)
    audio_queue = queue.Queue()
    state = {"in_speech": False}
    speech_buffer = []
    pre_buffer = collections.deque()
    pre_buf_max = target_rate * 800 // 1000
    pre_buf_size = [0]
    running = threading.Event()
    running.set()

    def on_audio(data):
        try:
            samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
            if args.channels > 1:
                samples = samples.reshape(-1, args.channels).mean(axis=1)
            if resampler is not None:
                samples = resampler.process(samples)
            if len(samples) == 0:
                return

            result = vad.detect(samples, target_rate)
            if result is None:
                return
            if result.is_speech_start:
                state["in_speech"] = True
                speech_buffer.clear()
                if pre_buffer:
                    speech_buffer.append(np.concatenate(list(pre_buffer)))
                speech_buffer.append(samples.copy())
                print("[VAD] speech detected", flush=True)
            elif state["in_speech"] and not result.is_speech_end:
                speech_buffer.append(samples.copy())
            elif result.is_speech_end and state["in_speech"]:
                speech_buffer.append(samples.copy())
                audio_queue.put(np.concatenate(speech_buffer))
                speech_buffer.clear()
                state["in_speech"] = False

            if not state["in_speech"]:
                pre_buffer.append(samples.copy())
                pre_buf_size[0] += len(samples)
                while pre_buf_size[0] > pre_buf_max:
                    pre_buf_size[0] -= len(pre_buffer.popleft())
        except Exception as error:
            print(f"[ERROR] audio callback: {error}", flush=True)

    spacemit_audio.init(
        sample_rate=args.rate,
        channels=args.channels,
        chunk_size=args.rate * args.channels * 2 // 25,
        capture_device=args.device,
    )
    capture = AudioCapture()
    capture.set_callback(on_audio)
    capture.start()
    print(f"Listening on device {args.device}; Ctrl+C to exit")

    def stop_handler(_signal, _frame):
        running.clear()
        audio_queue.put(None)

    previous_int = signal.signal(signal.SIGINT, stop_handler)
    previous_term = signal.signal(signal.SIGTERM, stop_handler)
    try:
        while running.is_set():
            try:
                audio = audio_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if audio is None:
                break
            try:
                result = asr.recognize(audio)
                if result and not result.is_empty:
                    print(f"[ASR] {result.text} (RTF={result.rtf:.2f})")
                    on_text(result.text)
            except Exception as error:
                print(f"[ERROR] recognition/control: {error}", flush=True)
    finally:
        capture.stop()
        asr.shutdown()
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)


def main():
    parser = argparse.ArgumentParser(description="LingLong standing action voice control")
    parser.add_argument("-d", "--device", type=int, default=-1)
    parser.add_argument("-r", "--rate", type=int, default=16000)
    parser.add_argument("-c", "--channels", type=int, default=2)
    parser.add_argument("-l", "--list-devices", action="store_true")
    parser.add_argument("--zmq-endpoint", default="tcp://127.0.0.1:5565")
    parser.add_argument("--zmq-timeout-ms", type=int, default=1000)
    parser.add_argument("--match-threshold", type=float, default=0.72)
    parser.add_argument("--cooldown-s", type=float, default=3.0)
    parser.add_argument("--wake-word", default="")
    parser.add_argument("--dry-run", action="store_true",
                        help="print matches without sending ZMQ commands")
    parser.add_argument("--text", help="match one sentence without a microphone")
    args = parser.parse_args()
    if not 0.0 < args.match_threshold <= 1.0:
        parser.error("--match-threshold must be in (0, 1]")
    if (not math.isfinite(args.cooldown_s) or args.cooldown_s < 0 or
            args.zmq_timeout_ms <= 0):
        parser.error("cooldown and ZMQ timeout must be nonnegative/positive")
    if args.text is not None and args.list_devices:
        parser.error("--text and --list-devices cannot be combined")

    client = None if args.dry_run or args.list_devices else HmiClient(
        args.zmq_endpoint, args.zmq_timeout_ms)
    last_sent_at = [float("-inf")]

    def handle_text(text):
        matched = match_action(text, args.match_threshold, args.wake_word)
        if matched is None:
            print(f"[SKIP] no unambiguous action: {text}")
            return
        action, score = matched
        print(f"[MATCH] {text} -> {action} ({score:.2f})")
        if args.dry_run:
            return
        now = time.monotonic()
        if now - last_sent_at[0] < args.cooldown_s:
            print("[SKIP] action cooldown")
            return
        if request_action(client, action):
            last_sent_at[0] = now

    try:
        if args.text is not None:
            try:
                handle_text(args.text)
            except Exception as error:
                parser.exit(1, f"[ERROR] {error}\n")
        else:
            run_audio(args, handle_text)
    finally:
        if client is not None:
            client.close()


if __name__ == "__main__":
    main()
