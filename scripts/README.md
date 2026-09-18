# ASR 使用

```bash
pip config set global.index-url https://mirrors.aliyun.com/pypi/simple/
pip config set global.extra-index-url https://git.spacemit.com/api/v4/projects/33/packages/pypi/simple
```

```bash
sudo apt install python3-venv python3-pip
python3 -m venv ~/audio_env
source ~/audio_env/bin/activate
pip install spacemit-ort opencv-python spacemit-vision spacemit-audio spacemit-vad spacemit-asr pyzmq
```

查看可用设备

```bash
arecord -l # 对应card编号
cat /proc/asound/card0/stream0 # 查看麦克风支持的格式、采样率和通道数
```

启动ASR

```bash
python scripts/asr_simple.py -d 0 -r 48000
```

## 站立动作语音控制

先启动 `humanoid_cmd_vel_hmi_node`，并让机器人进入 `stand_mjlab` 策略的
`RL` 状态。语音脚本只在 HMI 报告在线、无故障、站立策略已生效且没有切换或
进行中的交互动作时发送请求，不会自动上电或切换到站立模型。

可先不接麦克风测试识别映射：

```bash
python scripts/asr_action_control.py --text '请挥挥手' --dry-run
python scripts/asr_action_control.py --text '左手比心' --dry-run
```

实时监听并通过 ZMQ 请求动作：

```bash
python scripts/asr_action_control.py -d 0 -r 48000 \
  --zmq-endpoint tcp://127.0.0.1:5565
```

可用 `--wake-word 机器人` 要求每条指令包含唤醒词，例如“机器人挥手”。
`--match-threshold` 默认 `0.72`；匹配有歧义时不会发动作。
`--cooldown-s` 默认 `3` 秒，避免短时间重复触发。ZMQ 请求超时默认
`1000` 毫秒，可用 `--zmq-timeout-ms` 调整。返回“queued”只表示 HMI
接受排队请求，动作实际执行情况以 Control 状态回传为准。

脚本也安装为 ROS 2 可执行程序，可在构建并加载工作区后用
`ros2 run humanoid asr_action_control.py` 启动。
