# LingLong 语音动作脚本

[通用脚本与 ASR 环境](README.md) · [G1 感知脚本](README_g1.md) · [LingLong ROS2 主文档](../README_linglong.md)

本文说明 `asr_action_control.py` 的 ZMQ 动作控制，以及 `asr_action_control_websocket.py` 的 HMI 直连流程。
录音依赖、设备选择、SenseVoice／Qwen3-ASR 配置见[通用 README](README.md#共享-asr-后端与参数)。

先在 `spacemit_robot` 根目录加载环境，再进入本包目录运行下面的 Python 命令：

```bash
source build/envsetup.sh
cd middleware/ros2/control/humanoid
```

## ZMQ 动作控制

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

## WebSocket 直连 HMI

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
