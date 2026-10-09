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

直接通过 ZMQ 查询状态、执行或取消交互动作的消息格式见
[`ZMQ_ACTIONS.md`](ZMQ_ACTIONS.md)。

先启动 `humanoid_cmd_vel_hmi_node`。全身配置需要进入 `stand_mjlab/RL`，
`linglong_static.yaml` 固定底座配置需要进入 `TRAJECTORY`。语音脚本只在 HMI
报告在线、无故障、对应动作模式已生效且没有切换或进行中的交互动作时发送请求，
不会自动上电或切换状态；可先向节点发送 ZMQ `{"op":"stand"}` 自动推进状态机。

可先不接麦克风测试识别映射：

```bash
python scripts/asr_action_control.py --text '请挥挥手' --dry-run
python scripts/asr_action_control.py --text '左手比心' --dry-run
```

静态实机启动与无麦克风真实动作请求示例：

```bash
# 终端 1：固定底座核心进程
run_linglong.sh --real --profile static

# 终端 2：ROS/ZMQ operator 客户端
ros2 run humanoid humanoid_cmd_vel_hmi_node \
  "$SDK_ROOT/application/native/humanoid_linglong/config/linglong_static.yaml"

# 终端 3：先发送 {"op":"stand"} 并等待 TRAJECTORY 就绪，再发送文本动作
python scripts/asr_action_control.py --text '左手比心' --no-audio-prompts
```

实时监听并通过 ZMQ 请求动作：

```bash
python scripts/asr_action_control.py -d 0 -r 48000 \
  --zmq-endpoint tcp://127.0.0.1:5565
```

### WebSocket 直连 HMI

`asr_action_control_websocket.py` 直接连接 `hmi_runtime` 的 WebSocket API，
不需要启动 `humanoid_cmd_vel_hmi_node`，适合需要与 Web 控制端交替操作的
静态灵龙。它复用与 ZMQ 版本相同的 ASR、唤醒词、动作匹配和提示音逻辑，
不会修改或替代 `asr_action_control.py`。

脚本默认读取以下连接文件中的 WebSocket 地址和认证令牌：

```text
~/.local/state/humanoid-operator/linglong/connection.json
```

执行动作时，脚本会按需申请控制权、维持租约，并在动作结束后释放控制权。
若 Web 页面已持有控制权，本次动作会跳过并报告 `busy`，不会强制抢占。
Web 页面可以保持打开，但运行脚本前必须先在页面中释放控制权。脚本持有
控制权期间，Web 页面不能同时控制机器人。

静态模式未就绪时，脚本默认自动依次请求
`POWER_OFF → DAMP → HOME → ZERO → TRAJECTORY`。在 `ZERO` 中会等待
真实关节反馈满足 `zero_ready`，不会提前退回 `DAMP`。状态切换会使能并移动
机械臂，运行前必须固定机器人、清空运动范围并确保急停可用。

无麦克风执行一次动作：

```bash
python scripts/asr_action_control_websocket.py \
  --text '左手比心' \
  --no-audio-prompts
```

实时语音控制：

```bash
python scripts/asr_action_control_websocket.py -d 0 -r 48000
```

只测试识别映射，不连接 WebSocket、不申请控制权：

```bash
python scripts/asr_action_control_websocket.py \
  --text '左手比心' \
  --dry-run \
  --no-audio-prompts
```

常用 WebSocket 参数：

- `--connection-file`：连接文件路径，默认使用上述灵龙连接文件。
- `--socket-timeout-ms`：单次 WebSocket 请求超时，默认 `1500` 毫秒。
- `--mode-timeout-s`：进入 `TRAJECTORY` 的总超时，默认 `20` 秒。
- `--action-timeout-s`：等待动作完成的超时，默认 `30` 秒；超时会请求取消。
- `--poll-interval-s`：状态轮询周期，默认 `0.1` 秒。
- `--no-auto-enter-mode`：禁止自动切换模式；此时必须预先进入 `TRAJECTORY`。

WebSocket 版本仅使用 Python 标准库实现协议，不需要安装额外的 WebSocket
Python 包。若还需要 ROS `/cmd_vel` 控制，请继续使用 ZMQ/ROS 节点方案；
不要同时运行长期持有控制权的 `humanoid_cmd_vel_hmi_node` 和 WebSocket ASR。

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
都会播放“没听清楚”。播报时会保持录音设备打开但丢弃采集帧，避免
提示音被 ASR 再次识别，也避免某些 USB ALSA 设备重开后不再产生回调。
提示音会转换为默认的 48 kHz 双声道后播放，可用 `--playback-device`
选择输出设备，用 `--playback-rate` 和 `--playback-channels` 适配其他声卡，
用 `--no-audio-prompts` 关闭播报。
`--dry-run` 不发送 ZMQ 动作，但匹配成功时仍会播放“正在执行”；
匹配失败时不播放“没听清楚”。

两个动作控制脚本都安装为 ROS 2 可执行程序。构建并加载工作区后可使用：

```bash
ros2 run humanoid asr_action_control.py
ros2 run humanoid asr_action_control_websocket.py
```
