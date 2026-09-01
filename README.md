# 人形机器人导航与 RL 运控桥接

`humanoid` 是人形机器人 ROS2 控制集成包。当前提供
`humanoid_nav_rl_bridge_node`，用于将 ROS2 导航速度指令接入人形机器人的 RL
运控链路。核心节点只依赖通用的 `robot_base` 和 `transport_executor`；当前包内以
`linglong_` 开头的 launch 和参数文件是 LingLong 专用配置。桥接节点复用现有
人形机器人 `transport_executor` 的 HMI 角色，因此不需要修改 `control_runtime`、
`driver_runtime` 或 RL 策略代码。

## 数据流

```text
Nav2 /cmd_vel
  -> humanoid_nav_rl_bridge
  -> transport HMI 速度指令(vx, vy, wz)
  -> control_runtime / behavior_manager
  -> LingLong RL 行走策略
  -> 当前 DRIVER transport 端点
```

桥接节点也会接收 `control_runtime` 发布的 `ControlStatus`。只有控制端在线、
FSM 已进入 `RL` 状态，并且当前策略与 `auto_policy` 匹配时，节点才会发送
速度指令。

## 编译

需要先编译 `application/native/humanoid_common`，确保 `robot_base` 和
`transport_executor` 已安装到 `output/staging`，然后再编译本包：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select humanoid
source install/setup.bash
```

## 仅启动桥接节点

```bash
cd spacemit_robot
source build/envsetup.sh
```

终端 1：

```bash
run_driver_linglong.sh
```

终端 2：

```bash
run_control_linglong.sh
```

```bash
cd spacemit_robot

source output/staging/setup.zsh

ros2 launch humanoid linglong_nav_rl_bridge.launch.py \
  robot_config_path:=$PWD/application/native/humanoid_linglong/config/linglong.yaml auto_fsm:=true auto_policy:=walk_mjlab
```

设置 `auto_fsm:=true` 后，桥接节点会自动请求执行
`DAMP -> HOME -> ZERO -> RL` 状态切换流程。

## 启动同实例 MuJoCo、3D LiDAR、SLAM 与 Nav2

```bash
ros2 launch humanoid_mujoco linglong_lidar_bringup.launch.py \
  sdk_root:=/home/chenzhaoqi/k3-sdk-ext/spacemit_robot \
  auto_fsm:=true auto_policy:=walk_mjlab
```

该一体化 launch 会启动 `control_runtime` 和 ROS2 MuJoCo DRIVER 端点。
使用此方案时，不要同时启动原生 `driver_runtime`，否则两个 DRIVER 端点会发生冲突。
