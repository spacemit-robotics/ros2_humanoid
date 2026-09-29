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

默认 ASR 后端是进程内运行的 SenseVoice。可以使用 `--asr-backend`
切换到针对 K3 优化的 Qwen3-ASR 0.6B：

```bash
sudo apt update
sudo apt install -y llama.cpp-tools-spacemit
dpkg-query -W llama.cpp-tools-spacemit

mkdir -p ~/.cache/models/asr/qwen3asr
cd ~/.cache/models/asr/qwen3asr
wget -O qwen3-asr-0.6B-dynq-q40.tar.gz \
  https://archive.spacemit.com/spacemit-ai/model_zoo/asr/qwen3-asr-0.6B-dynq-q40.tar.gz
tar -xzf qwen3-asr-0.6B-dynq-q40.tar.gz
rm -f qwen3-asr-0.6B-dynq-q40.tar.gz
```

Qwen3-ASR 媒体后端要求 `llama.cpp-tools-spacemit` 版本不低于 `0.1.7`。
模型目录默认是
`~/.cache/models/asr/qwen3asr/qwen3-asr-0.6B-dynq-q40`，其中必须包含：

```text
Qwen3-ASR-0.6B-text-q40.gguf
Qwen3-ASR-0.6B-encoder-backend.dynq.onnx
Qwen3-ASR-0.6B-encoder-frontend.dynq.onnx
```

使用本机 Qwen3-ASR：

```bash
python asr_action_control.py \
  --asr-backend qwen3_asr \
  -d 0 -r 48000 -c 1 \
  --playback-device 1 \
  --wake-word 小龙
```

使用, 在没有 zmq 服务时进行语音链路验证：

```bash
--dry-run
```

如需显式指定播放格式：

```bash
--playback-rate 48000 --playback-channels 2
```

脚本默认检查 `http://127.0.0.1:8063/health`。端点尚未运行时，它会使用
本机模型自动启动 `llama-server`，退出时只停止由本次脚本启动的服务。服务日志写入
`/tmp/humanoid_qwen3_asr.log`。可用以下参数覆盖默认配置：

- `--qwen3-model-dir`：本机模型目录。
- `--qwen3-endpoint`：OpenAI 兼容的 `/v1/chat/completions` 地址。
- `--qwen3-server-threads`：自动启动服务时的推理线程数，默认 `4`。
- `--qwen3-timeout-sec`：单次识别请求超时，默认 `10` 秒。
- `--qwen3-startup-timeout-sec`：等待本机服务启动的超时，默认 `90` 秒。
- `--qwen3-no-auto-start`：不自动启动服务，用于连接已运行的本机或远程服务。

使用另一台设备上的 Qwen3-ASR 服务：

```bash
python scripts/asr_action_control.py \
  --asr-backend qwen3_asr \
  --qwen3-endpoint http://192.168.1.20:8063/v1/chat/completions \
  --qwen3-no-auto-start \
  -d 0 -r 48000 -c 2 \
  --zmq-endpoint tcp://127.0.0.1:5565
```

Qwen3-ASR 接收经过 VAD 分段的 16 kHz 单声道 WAV，每个语音段完成后输出整句
文字，不是逐字流式识别。脚本会把全部动作短语和可选唤醒词作为识别上下文。
多通道录音默认取各通道平均值；麦克风阵列若存在相位抵消，应优先把 `-c` 设置成
设备实际可用的单通道模式。

可用 `--wake-word 机器人` 要求每条指令包含唤醒词，例如“机器人挥手”。
`--match-threshold` 默认 `0.72`；匹配有歧义时不会发动作。
`--cooldown-s` 默认 `3` 秒，避免短时间重复触发。ZMQ 请求超时默认
`1000` 毫秒，可用 `--zmq-timeout-ms` 调整。返回“queued”只表示 HMI
接受排队请求，动作实际执行情况以 Control 状态回传为准。
VAD 分段默认最长 `8` 秒；持续噪声导致检测不到语音结束时，会强制
结束当前分段，可用 `--max-utterance-s` 调整。

动作请求排队成功后会播放“正在执行”；包含唤醒词但无法唯一匹配
动作时会播放“没听清楚”。未配置唤醒词时，任何非空 ASR 结果匹配失败
都会播放“没听清楚”。播报时会暂停录音，避免提示音被 ASR 再次识别。
提示音会转换为默认的 48 kHz 双声道后播放，可用 `--playback-device`
选择输出设备，用 `--playback-rate` 和 `--playback-channels` 适配其他声卡，
用 `--no-audio-prompts` 关闭播报。
`--dry-run` 不发送 ZMQ 动作，但匹配成功时仍会播放“正在执行”；
匹配失败时不播放“没听清楚”。

脚本也安装为 ROS 2 可执行程序，可在构建并加载工作区后用
`ros2 run humanoid asr_action_control.py` 启动。
