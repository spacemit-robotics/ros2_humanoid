# 人形机器人 ROS2 控制

## 项目简介

`humanoid` 是人形机器人 ROS2 控制集成包。当前提供
`humanoid_nav_rl_bridge_node`，用于将 Nav2 `/cmd_vel` 速度指令接入现有 RL
运控链路，复用 `transport_executor` 的 HMI 通道，无需修改 `control_runtime`、
`driver_runtime` 或策略实现。

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
