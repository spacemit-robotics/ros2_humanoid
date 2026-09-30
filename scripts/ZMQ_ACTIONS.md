# 交互动作 ZMQ 接口

本文档说明 `humanoid_cmd_vel_hmi_node` 当前用于触发 LingLong 站立交互动作的
ZMQ 消息格式。

## 连接方式

- 默认端点：`tcp://127.0.0.1:5565`
- 模式：ZMQ `REQ/REP`
- 编码：UTF-8 JSON
- 每发送一个请求，必须接收对应响应后才能继续使用同一个 `REQ` socket。
- 服务端由 `humanoid_cmd_vel_hmi_node` 提供，可通过 ROS 参数
  `zmq_endpoint` 修改监听地址。

## 执行动作的前置条件

发送交互动作前，机器人需要满足：

- Control 在线且 HMI 命令通道已连接；
- 没有锁存故障；
- FSM 当前处于 `RL`；
- 当前策略为 `stand_mjlab`；
- 没有进行策略切换；
- 没有其他交互动作处于“进入”“播放”“保持”或“收回”阶段。

建议先发送状态查询：

```json
{"op":"status"}
```

响应示例：

```json
{
  "ok": true,
  "online": true,
  "mode": "RL",
  "policy": "stand_mjlab",
  "desired_policy": "stand_mjlab",
  "switching": false,
  "interaction_phase": "就绪",
  "fault": false
}
```

可以发送新动作的 `interaction_phase` 通常为“就绪”“完成”或“拒绝”。“进入”、
“播放”、“保持”和“收回”表示已有动作正在执行。

## 发送交互动作

通用格式：

```json
{"op":"interaction","action":"wave_hello"}
```

请求被 HMI 接受时的响应示例：

```json
{
  "ok": true,
  "action": "wave_hello",
  "message": "请求播放交互动作 → 挥手"
}
```

`ok: true` 仅表示 HMI 已接受并排队该请求，不表示动作已经播放完成。实际执行状态
应继续通过 `{"op":"status"}` 的 `interaction_phase` 观察。

## 可用 action

以下 action 来自 `linglong.yaml` 的 `upper_body_actions.action_names`，当前仅用于
`stand_mjlab`：

| action | 动作 |
| --- | --- |
| `wave_hello` | 挥手 |
| `wave_under_head` | 胸前挥手 |
| `wave_above_head` | 高举挥手 |
| `heart_both` | 比心 |
| `heart_left` | 左手比心 |
| `heart_right` | 右手比心 |
| `shake_hand` | 右手握手 |
| `shake_hand_left` | 左手握手 |
| `blow_kiss_with_left_hand` | 左手飞吻 |
| `blow_kiss_with_right_hand` | 右手飞吻 |
| `blow_kiss_with_both_hands` | 双手飞吻 |
| `hug` | 欢迎 |
| `high_five` | 击掌 |
| `right_hand_up` | 右手摊手 |
| `both_hands_up` | 双手摊手 |
| `box_left_hand_win` | 胜利庆祝 |
| `right_hand_on_heart` | 抚胸致意 |
| `ultraman_ray` | 奥特曼光线 |

action 必须使用表中的英文键，不能发送中文显示名称。

## 取消动作

请求平滑取消当前动作：

```json
{"op":"cancel"}
```

响应示例：

```json
{
  "ok": true,
  "message": "请求平滑取消交互动作"
}
```

如果当前没有可取消的动作，响应中的 `ok` 为 `false`，`message` 会说明当前状态。

## 常见失败响应

action 不存在或没有在 `stand_mjlab` 注册：

```json
{"ok":false,"error":"action is not available on stand_mjlab"}
```

机器人状态不允许执行站立动作：

```json
{"ok":false,"error":"stand_mjlab/RL is not ready"}
```

动作已经在执行、Control 拒绝请求或其他运行时条件不满足时，响应通常包含：

```json
{
  "ok": false,
  "action": "wave_hello",
  "message": "已有动作执行中，请先按 C 平滑取消"
}
```

无法解析的 JSON、非对象请求或未知 `op` 会返回 `ok: false` 和对应的 `error`。

## Python 示例

需要安装 `pyzmq`：

```python
import time

import zmq


endpoint = "tcp://127.0.0.1:5565"
context = zmq.Context.instance()
socket = context.socket(zmq.REQ)
socket.setsockopt(zmq.LINGER, 0)
socket.setsockopt(zmq.SNDTIMEO, 1000)
socket.setsockopt(zmq.RCVTIMEO, 1000)
socket.connect(endpoint)

socket.send_json({"op": "status"})
status = socket.recv_json()
ready = (
    status.get("ok")
    and status.get("online")
    and not status.get("fault")
    and status.get("mode") == "RL"
    and status.get("policy") == "stand_mjlab"
    and not status.get("switching")
    and status.get("interaction_phase")
    not in {"进入", "播放", "保持", "收回"}
)
if not ready:
    raise RuntimeError(f"robot is not ready: {status}")

socket.send_json({"op": "interaction", "action": "wave_hello"})
reply = socket.recv_json()
if not reply.get("ok"):
    raise RuntimeError(f"action rejected: {reply}")
print(reply)

# 可选：轮询观察动作阶段。
for _ in range(100):
    time.sleep(0.1)
    socket.send_json({"op": "status"})
    status = socket.recv_json()
    print(status["interaction_phase"])
    if status["interaction_phase"] in {"完成", "拒绝"}:
        break
```

生产代码应像 `asr_action_control.py` 一样在超时后丢弃当前 `REQ` socket 并新建
socket，避免 ZMQ `REQ` 状态机因缺少响应而无法发送后续请求。

