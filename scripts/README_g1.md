# G1 感知脚本

[通用脚本](README.md) · [LingLong 脚本](README_linglong.md) · [G1 ROS2 主文档](../README_g1.md)

数据链路：**RealSense → 大脑 ROS2/ZMQ 桥接 → 小脑 Hiking**。查看器独立订阅，关闭窗口不影响控制。
以下命令默认在 `spacemit_robot` 根目录执行；大脑 SDK 路径为 `/home/bianbu/spacemit_robot`。

## 仿真：启动并查看深度

```bash
# 终端 1：仿真与控制，自动使用 simulation.perception
output/staging/bin/run_g1.sh --sim --stairs

# 终端 2：查看深度，无需 ROS 环境
python3 middleware/ros2/control/humanoid/scripts/view_hiking_depth.py
```

查看器需要图形桌面及 `python3-zmq python3-numpy python3-opencv`。
左图显示原始深度，白框标出模型裁剪范围；右图需要 OpenCV ≥ 4.10，显示裁剪、补洞和滤波后的 `32×18` 深度。
红色近、蓝色远，原图中的空洞为黑色。常用参数：`--raw-only` 只看原图，`--fps 5` 降低刷新率，
`--max-depth 5` 将色标上限设为 5 m。按 `Q`、`Esc` 或关闭窗口退出。

## 实机：手动启动

小脑需已运行 `g1.service`，或手动以 `run_g1.sh --real` 启动 native 服务，两种方式选一。
启动时自动使用顶层 `perception` 配置。
`5561` 时间锚点服务随小脑 driver 启动；大脑桥接通过它对齐相机与机器人时间。
大脑需要 ROS Humble、RealSense 驱动、`rclpy sensor_msgs python3-numpy python3-zmq`。

```bash
# 大脑终端 1：RGB、深度均为 640×360@30，开启对齐、同步和 IMU
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch realsense2_camera rs_launch.py camera_namespace:=/ \
  enable_color:=true enable_depth:=true \
  rgb_camera.color_profile:=640,360,30 \
  depth_module.depth_profile:=640,360,30 \
  align_depth.enable:=true enable_sync:=true \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2

# 大脑终端 2：正式深度桥接，自动启动时间同步线程
source /opt/ros/humble/setup.bash
cd /home/bianbu/spacemit_robot
python3 middleware/ros2/control/humanoid/scripts/realsense_depth_zmq.py \
  --mode control --clock-endpoint tcp://192.168.60.153:5561
```

首次同步约需 5 s。看到 `[clock] ready` 和 `mode=control, state=ready` 后，再由操作者切换到 Hiking。
不带 `--mode control` 时默认为预览模式，不能用于 Hiking 控制。
使用 `--clock-endpoint` 后，无需单独运行 `g1_clock_sync.py`。

## 连接地址与 PC 查看

桥接默认 `ROS_DOMAIN_ID=73`，订阅 `/camera/depth/image_rect_raw` 和 `/camera/depth/camera_info`。

| 用途 | 地址 |
| --- | --- |
| 大脑发布深度，监听有线与 Wi-Fi | `tcp://0.0.0.0:5560`（仅用于绑定） |
| 小脑订阅大脑深度 | `tcp://192.168.60.1:5560` |
| PC 通过大脑 Wi-Fi 查看 | `tcp://10.0.91.145:5560` |
| 大脑本机查看 | `tcp://127.0.0.1:5560` |
| 大脑查询小脑时间锚点 | `tcp://192.168.60.153:5561` |

在有图形桌面的 PC 上运行：

```bash
python3 middleware/ros2/control/humanoid/scripts/view_hiking_depth.py \
  --endpoint tcp://10.0.91.145:5560
```

Wi-Fi 地址不可直达时，可建立 `ssh -N -L 15560:192.168.60.1:5560 bianbu@10.0.91.145`，
再连接 `tcp://127.0.0.1:15560`。大小脑之间的控制深度走有线网。

桥接将原始深度映射为 SDK 相机投影的 `64×36`、58.29° 深度，单位为米，保留光轴 Z 距离和空洞。
它按实际到帧发布，约 30 FPS；裁剪、补洞和滤波由 control 完成，策略按 50 Hz 推理。

## 大脑开机自启

使用两个服务代替手动启动，两种方式选一。默认用户为 `bianbu`，SDK 位于 `/home/bianbu/spacemit_robot`。

| 服务 | 功能 |
| --- | --- |
| `g1-realsense.service` | 按上述相机基线采集 RGB、深度和 IMU |
| `g1-hiking-depth.service` | 启动正式桥接和时间同步，同时拉起相机服务 |

首次安装：

```bash
cd /home/bianbu/spacemit_robot
sudo install -m 644 middleware/ros2/control/humanoid/scripts/systemd/g1-*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now g1-realsense.service g1-hiking-depth.service
```

日常管理：

```bash
systemctl status g1-realsense.service g1-hiking-depth.service
journalctl -u g1-realsense.service -u g1-hiking-depth.service -f
sudo systemctl stop g1-hiking-depth.service g1-realsense.service
sudo systemctl start g1-realsense.service g1-hiking-depth.service
# 取消开机自启并停止
sudo systemctl disable --now g1-hiking-depth.service g1-realsense.service
```

**停服前先退出 Hiking RL；服务运行时不要再手动启动同一相机或桥接。**
服务只负责数据采集和发布，不申请控制权或自动进入 RL。主进程退出后 5 s 重试。
服务的时间映射保存在 `/run/g1-hiking-depth/clock_map.json`；更换用户或 SDK 路径时需修改 unit。

## 状态与排查

| 日志／界面 | 含义与检查方向 |
| --- | --- |
| `[clock] ready` | 时间映射就绪；首次约需 5 s，后续持续刷新 |
| `mode=control, state=ready` | 正式深度正在发布；关注约 30 FPS、覆盖率 100% 和帧龄 |
| `PREVIEW ONLY` | 预览流，使用 ROS 时间；正式控制需添加 `--mode control --clock-endpoint …` |
| `STALE` | 超过 0.2 s 未收到新帧；检查相机、桥接进程和网络 |
| 时间映射缺失／过期 | 检查小脑 driver 和 `5561` 端口；映射恢复前不发布控制深度 |

正式模式要求映射年龄不超过 2 s、误差预算不超过 20 ms、采集帧龄不超过 0.2 s，且投影覆盖率为 100%。
输入或映射不满足条件时停止发布；发布中断后恢复时重新积累深度历史，不自动降级为预览流。
误差预算包含配置的 10 ms IMU 接收延迟假设，NTP 同步不能替代机器人时间映射。

`queued` 只表示本地 PUB 入队次数；查看器默认最多刷新 10 FPS，`source Hz` 才是采集频率估计。
`Last received` 是本机距上次收帧的时长，不是端到端延迟。更多参数见各脚本的 `--help`。
