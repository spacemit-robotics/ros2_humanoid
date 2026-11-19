# Hiking 深度查看

在 `spacemit_robot` 目录，先启动仿真，再在另一个终端运行查看器。
若 `g1.yaml` 已使用实机深度配置，先按 `application/native/humanoid_unitree_g1/docs/perceptive_control.md`
将深度发布与时钟切回仿真设置：

```bash
# 终端 1：仿真与控制
output/staging/bin/run_g1.sh --sim --stairs

# 终端 2：只读订阅深度，无需 ROS 环境
python3 middleware/ros2/control/humanoid/scripts/view_hiking_depth.py
```

依赖为 `python3-zmq python3-numpy python3-opencv`，需要图形桌面。
默认显示原始深度和 Hiking 预处理后的裁剪区域，白框表示原图中的裁剪范围。
红色近、蓝色远，原始无效深度为黑色；模型输入中的无效值按策略规则填为 2.5 m。
使用 `--raw-only` 只看原图，`--fps 5` 降低刷新率，`--max-depth 5` 调整色标上限。
`--endpoint tcp://发布机地址:5560` 可连接已开放网络监听的发布器。

查看器使用独立 ZMQ SUB，只保留最新帧，默认最多刷新 10 FPS；退出窗口不影响控制。
`source Hz` 根据帧序号和采集时间估算；`Last received` 表示本机距离上次接收的时长，
不代表采集到显示的端到端延迟。超过 0.2 s 未收到新帧显示 `STALE`。
按 `Q`、`Esc`、`Ctrl-C` 或关闭窗口退出。

# 大脑 RealSense ROS2 → ZMQ

相机基线为彩色、深度均 `640×360@30`，对齐、同步和 IMU 均开启：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch realsense2_camera rs_launch.py camera_namespace:=/ \
  enable_color:=true enable_depth:=true \
  rgb_camera.color_profile:=640,360,30 \
  depth_module.depth_profile:=640,360,30 \
  align_depth.enable:=true enable_sync:=true \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2
```

相机已运行时，在另一个终端启动桥接（无需重新编译）：

```bash
source /opt/ros/humble/setup.bash
cd /home/bianbu/spacemit_robot
python3 middleware/ros2/control/humanoid/scripts/realsense_depth_zmq.py
```

脚本默认使用 `ROS_DOMAIN_ID=73`，订阅 `/camera/depth/image_rect_raw` 和
`/camera/depth/camera_info`，PUB 默认绑定 `tcp://0.0.0.0:5560`，同时监听有线和 Wi-Fi。
小脑连接 `tcp://192.168.60.1:5560`，PC 可直接连接 `tcp://10.0.91.145:5560`，
大脑本机可使用 `tcp://127.0.0.1:5560`；订阅者应填写具体地址，不能填写 `0.0.0.0`。
依赖 `rclpy sensor_msgs python3-numpy python3-zmq`，桥接不需要 OpenCV 或图形桌面。
`--bind`、`--domain-id`、`--depth-topic`、`--camera-info-topic` 可覆盖默认值；
`--duration 20` 用于限时验证。`16UC1` 默认按 ROS 毫米深度转换为米，
自定义发布源可用 `--depth-scale` 指定比例；`32FC1` 按米读取。

原始深度按 CameraInfo 的 rectified 投影矩阵映射至 `64×36`、垂直视场 58.29°，
使用最近邻采样并保留光轴 Z 距离；不改变真实光心/安装方向，不预先做 Hiking 裁剪或滤波。
相机视场以外及无效深度保持为零，并报告目标投影覆盖率。按相机实际到帧发布，
不复制帧凑 50 Hz；ROS 订阅队列为 1，ZMQ 发送队列为 2。

默认 `--mode preview` 用于采集和链路验证：`profile=g1-realsense-preview-v1`、
`clock=ros`、`control_ready=false`，保留真实光学坐标系和 ROS 采集时间；
Hiking 控制接收器会拒绝预览 profile。相机安装采用已确认与仿真一致的 SDK 位姿。

正式模式使用小脑 driver 的只读时钟服务，推荐直接启动内置同步线程：

```bash
source /opt/ros/humble/setup.bash
python3 middleware/ros2/control/humanoid/scripts/realsense_depth_zmq.py \
  --mode control --clock-endpoint tcp://192.168.60.153:5561
```

大脑可安装两个 systemd 服务，配置与上述相机基线一致（默认用户 `bianbu`、SDK 位于 `/home/bianbu/spacemit_robot`）：

```bash
cd /home/bianbu/spacemit_robot
sudo install -m 644 middleware/ros2/control/humanoid/scripts/systemd/g1-*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now g1-realsense.service g1-hiking-depth.service
```

`g1-realsense.service` 启动相机；`g1-hiking-depth.service` 启动正式深度桥接和内置时间同步，
并拉起相机服务。两者均以 `bianbu` 运行，设置 `ROS_DOMAIN_ID=73`，无需登录或手工 source。
主进程退出后 5 s 重试；相机未就绪或小脑 `5561` 暂不可达时，桥接等待有效输入，
不会发布预览包代替正式深度。网络恢复后自动重建映射与深度流。
时间映射放在私有目录 `/run/g1-hiking-depth/clock_map.json`，无需再运行独立同步脚本。

```bash
systemctl status g1-realsense.service g1-hiking-depth.service
journalctl -u g1-realsense.service -u g1-hiking-depth.service -f
# 停止／恢复两项服务；停相机和桥接前须先退出 Hiking RL
sudo systemctl stop g1-hiking-depth.service g1-realsense.service
sudo systemctl start g1-realsense.service g1-hiking-depth.service
# 取消开机自启并停止
sudo systemctl disable --now g1-hiking-depth.service g1-realsense.service
```

服务启动只采集和发布数据，不会接管控制权或自动进入 RL。运行服务时不要再手工启动同一相机或桥接。
更换安装用户或 SDK 目录时，需修改 unit 的 `User`、`Group`、`WorkingDirectory` 和 `ExecStart`。

小脑需更新并重编译 `humanoid_common`，在 `g1.yaml` 设置：

```yaml
perception:
  depth:
    publish_sim: false
    endpoint: "tcp://192.168.60.1:5560"
    clock: robot
    timeout_s: 0.2
  clock:
    publish: true
    endpoint: "tcp://0.0.0.0:5561"
    receive_delay_bound_s: 0.01
```

小脑 driver 从 HI91 诊断数据读取设备采样时间和主机串口接收时间，每 50 ms 更新锚点，
独立后台 ZMQ REP 线程提供只读查询。大脑以 10 Hz 发起四时间戳请求，估计网络时延和
两机 monotonic 偏移，再拟合 HI91 与 ROS 系统时钟的速率。无需读取小脑串口或订阅完整深度 ROS。
默认至少收集 5 s、30 个锚点才发布有效映射；同步不会申请控制权或改变机器人状态。

`receive_delay_bound_s` 是假设的采样到主机接收延迟上限，默认预算 10 ms；
估计器按区间中点补偿，并把半区间、网络往返、拟合残差和外推预算计入误差。
网络测试无法证明 IMU 的绝对单向延迟上限；更换 IMU、串口或驱动后应重新确认该预算。
NTP 同步不能代替 HI91 设备时钟映射。

同步线程原子更新 `/run/user/<uid>/g1_clock_map.json`（优先使用 `XDG_RUNTIME_DIR`），
`--time-map` 可另选路径。文件使用独占写锁、权限 0600；失效响应立即删除映射，
连续无有效更新超过 0.5 s 也删除；退出时清理。设备重启或 ROS 时钟跳变后重新积累锚点。
控制模式不会在等待映射期间发送深度。

也可单独运行 `g1_clock_sync.py --endpoint tcp://192.168.60.153:5561`，
桥接用 `--mode control --time-map /run/user/<uid>/g1_clock_map.json` 读取外部映射，
此时不传 `--clock-endpoint`。不要让两个同步程序写同一个文件。
映射包含 `version=1`、`source_clock=ros`、`target_clock=robot`、`session`、
`source_time_s`、`target_time_s`、`scale`、`updated_ros_time_s`、`error_bound_s`。
公式为 `robot_time = target_time_s + (image_ros_time - source_time_s) * scale`。
设备重启或映射不连续时更换 `session`，正常刷新保持该值不变。

正式模式仅在以下条件满足时发布 `g1-sdk-hiking-v1`、`clock=robot` 的原生控制消息：

- 映射默认不超过 2 s，误差上界不超过 20 ms，比例漂移不超过 ±1000 ppm。
- 采集帧龄默认不超过 0.2 s，不接受超过 20 ms 的未来帧；映射后的时间递增。
- 目标投影覆盖率为 100%，有有效深度，源图像和内参的尺寸／光学坐标系一致。
- 使用 `torso_link/rgbd_camera`、SDK 的 `camera_pos` 和 `camera_xyaxes`；
  真实源坐标系另外保留在 `source_frame`。不会将彩色对齐深度冒充原始深度。

映射缺失、损坏、过期或不满足限制时，正式模式停止发布，不自动降级发预览消息。
相机断流超过 0.2 s 后恢复、ROS 时钟复位、内参变化或映射会话改变时，重建流会话；
控制发布中断后恢复也更换会话，让接收端重新积累历史。单个重复／乱序帧直接丢弃。
无效 CameraInfo 会清除旧投影，不能继续使用旧内参。
这些检查表示输入满足接口条件，不表示机器人已经允许进入 RL。

每 5 s 输出近期发布帧率、采集帧龄 P95、投影覆盖率、有效深度比例和丢弃计数。
`queued` 仅表示提交给本地 PUB 的次数，不表示订阅端已接收；接收端仍应自行监测时效。
参数及限制见 `--help`。无 ROS 的逻辑回归可运行
`python3 middleware/ros2/control/humanoid/tests/test_realsense_depth_zmq.py`，
时间拟合与断链回归使用同目录的 `test_g1_clock_sync.py`。

在有图形桌面的 PC 上直接查看 Wi-Fi 地址：

```bash
python3 middleware/ros2/control/humanoid/scripts/view_hiking_depth.py \
  --endpoint tcp://10.0.91.145:5560
```

外部电脑也可先建立 `ssh -N -L 15560:192.168.60.1:5560 bianbu@10.0.91.145`，
再将查看器端点设为 `tcp://127.0.0.1:15560`。这只是可视化通道，
未来大小脑之间的控制深度直接走 `192.168.60.0/24` 有线网。

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
