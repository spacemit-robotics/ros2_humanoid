# 人形机器人 ROS2 控制

## 项目简介

`humanoid` 是人形机器人 ROS2 控制集成包，用于把 ROS2 导航、终端控制和 TF
接入现有 native 人形机器人运控链路。当前提供三个节点：

- `humanoid_nav_rl_bridge_node`：将 Nav2 `/cmd_vel` 速度指令接入 RL 运控链路，用于mujoco一键仿真。
- `humanoid_cmd_vel_hmi_node`：保留原生 HMI 的界面与 FSM 操作，同时接收 ROS2
  `/cmd_vel`，用于实机 ros2 控制机器人。
- `humanoid_head_tf_node`：只读获取 LingLong 头部 yaw、pitch 实机关节角度，发布
  虚拟 `base_link`、头部动态 TF 和可配置的相机静态 TF，无需 URDF，用于实机建图避障场景。

节点复用现有 `transport_executor` 或只读观察其 SHM 状态，不修改
`control_runtime`、`driver_runtime` 或策略实现。

## 功能特性

- 支持将 ROS2 `Twist` 转换为人形机器人纵向、横向和转向速度指令。
- 支持自动选择 RL 策略并请求 FSM 进入 RL 状态。
- 支持速度限幅、加速度限制、指令超时清零和 RL 状态门控。
- 支持通过状态话题观察控制连接、FSM、策略和速度门控状态。
- 核心节点可复用于不同人形机器人，当前仅提供 LingLong 专用 launch 和参数。
- 本包不提供底层 driver、`control_runtime`、RL 策略模型或 Nav2。

## 快速开始

以下流程在 `spacemit_robot` 根目录执行。

### 环境准备

准备 ROS2 Humble，并先构建 `application/native/humanoid_common`，确保
`robot_base` 和 `transport_executor` 已安装到 `output/staging`。

### 构建编译

```bash
source build/envsetup.sh
lunch k3-com260-kit-humanoid-linglong
m -C
m -R
```

`m -C` 构建 native underlay，`m -R` 构建包括 `humanoid` 在内的 ROS2 包。

### 运行示例

先启动匹配的 driver 和 `control_runtime`，再启动桥接节点：

```bash
source /opt/ros/humble/setup.bash
source output/staging/setup.bash

ros2 launch humanoid linglong_nav_rl_bridge.launch.py \
  robot_config_path:=$PWD/application/native/humanoid_linglong/config/linglong.yaml \
  auto_fsm:=true \
  auto_policy:=walk_mjlab
```

`auto_fsm:=true` 会自动请求执行
`POWER_OFF -> DAMP -> HOME -> ZERO -> RL`。可通过以下命令检查状态：

```bash
ros2 topic echo /humanoid_nav_rl_bridge/status
```

## 详细使用

数据流如下：

```text
Nav2 /cmd_vel
  -> humanoid_nav_rl_bridge
  -> transport HMI command
  -> control_runtime / behavior_manager
  -> RL locomotion policy
  -> DRIVER transport endpoint
```

节点仅在控制端在线、FSM 处于 `RL` 且当前策略与 `auto_policy` 匹配时转发速度。
参数默认值见 [`config/linglong_nav_rl_bridge.yaml`](config/linglong_nav_rl_bridge.yaml)。
后续完整接口和部署方式以 SpacemiT Robot 官方文档为准。

## ROS2 cmd_vel 终端 HMI

`humanoid_cmd_vel_hmi_node` 是 `run_hmi_linglong.sh` 的 ROS2 输入版本。它复用
原 HMI 的终端界面、FSM 状态切换、策略选择、状态确认、心跳和退出保护逻辑，
并订阅 `/cmd_vel` 更新速度。节点不会通过 ROS2 发布机器人控制消息；控制仍由
原 HMI transport 发送给 `control_runtime`。

启动 driver 和 control 后，在交互式终端中用该节点替代
`run_hmi_linglong.sh`：

```bash
source output/staging/setup.zsh
ros2 run humanoid humanoid_cmd_vel_hmi_node \
  "$PWD/application/native/humanoid_linglong/config/linglong.yaml"
```

可选 ROS2 参数：

```bash
ros2 run humanoid humanoid_cmd_vel_hmi_node \
  "$PWD/application/native/humanoid_linglong/config/linglong.yaml" \
  --ros-args \
  -p cmd_vel_topic:=/cmd_vel \
  -p cmd_vel_timeout_s:=0.5
```

不要同时启动该节点和 `run_hmi_linglong.sh`，因为二者都是 HMI transport 端。

## LingLong 头部与相机 TF

`humanoid_head_tf_node` 以只读方式旁路观察 LingLong SHM 状态，不会消费
`control_runtime` 的状态数据，也不会发送控制命令。节点根据实机反馈的
`head_yaw_joint` 和 `head_pitch_joint` 发布：

```text
base_link -> head_yaw_link -> head_pitch_link -> camera_link
```

`base_link` 是该节点创建的 TF 根坐标系，不发布 `world` 或 `odom` 到
`base_link` 的变换。头部坐标关系为：

- `base_link -> head_yaw_link`：平移为 `[0, 0, 0.4168]` 米，旋转为
  `Rz(head_yaw_joint)`。即 yaw 关节绕位于 `base_link` 上方 0.4168 米处的 Z 轴
  旋转；yaw 为零时两坐标系方向一致。
- `head_yaw_link -> head_pitch_link`：平移为 `[0, 0, 0]`，旋转为
  `Ry(head_pitch_joint)`。pitch 为零时两个坐标系的原点和方向完全重合；pitch
  改变后原点仍重合，仅 `head_pitch_link` 绕共同原点的 Y 轴旋转。

旋转正方向遵循右手定则，关节角使用经过硬件极性和零偏标定后的弧度值。节点收到
有效实机关节反馈前不会发布上述动态 TF。`head_mount_xyz` 可覆盖默认的
`[0, 0, 0.4168]`。

相机外参是 `head_pitch_link -> camera_link` 静态 TF，默认关闭；
测量或标定后通过 `camera_xyz`（米）和 `camera_rpy`（弧度，roll/pitch/yaw）提供：

```bash
source output/staging/setup.zsh
ros2 run humanoid humanoid_head_tf_node \
  "$PWD/application/native/humanoid_linglong/config/linglong.yaml" \
  --ros-args \
  -p camera_tf_enabled:=true \
  -p camera_xyz:="[0.0, 0.0, 0.0]" \
  -p camera_rpy:="[0.0, 0.0, 0.0]"
```

把示例中的零外参替换为实测值。可选参数包括 `base_frame`、`yaw_frame`、
`pitch_frame`、`camera_frame`、`head_mount_xyz`、`publish_rate_hz` 和
`state_timeout_s`。该节点仅支持 `transport.type: shm`，应与 driver、control
运行在同一台机器并具有状态共享内存的读取权限。

启动后可分别检查动态头部 TF 和相机静态 TF：

```bash
ros2 run tf2_ros tf2_echo base_link head_yaw_link
ros2 run tf2_ros tf2_echo head_yaw_link head_pitch_link
ros2 run tf2_ros tf2_echo head_pitch_link camera_link
```

## 常见问题

- 状态显示 `control=offline`：确认 driver 和 `control_runtime` 已启动，并使用同一
  `robot_config_path`。
- 无法自动进入 RL：确认 `auto_fsm:=true`、策略模型存在且 `auto_policy` 名称有效。
- 收到 `/cmd_vel` 但机器人不动：确认状态为 `mode=RL`、策略匹配且
  `velocity_gate=open`。
- 启动 MuJoCo 一体化方案时发生 DRIVER 冲突：不要同时启动原生
  `driver_runtime`。
- 找不到 `robot_base` 或 `transport_executor`：先执行 `m -C` 构建 native
  underlay。

## 版本与发布

当前包版本和依赖信息以 `package.xml` 为准，主要兼容 ROS2 Humble。当前提供的
LingLong 参数和启动接口在正式发布前可能调整。

## 贡献方式

请从最新主分支创建功能或修复分支，确保构建、单元测试和 lint 检查通过后提交
Pull Request。贡献者与维护者名单见：`CONTRIBUTORS.md`。

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
