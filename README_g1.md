# Unitree G1 人形机器人 ROS2 控制

## 项目简介

`humanoid` 将 ROS2 `/cmd_vel` 接入 G1 的 native 运控链路，并提供实机腰部、
IMU 姿态动态 TF 和 D435i 安装 TF。LingLong 的导航桥接、语音动作和头部 TF 说明见
[README_linglong.md](README_linglong.md)。

- `g1_cmd_vel_bridge_node`：通过公开的 `operator_client` 接口，在已有 RL 状态下
  申请控制权并转发速度，不切换机器人状态或策略。
- `g1_fact_tf.launch.py`：按 G1 `rev_1_0` 模型发布固定机体姿态和 D435i 安装 TF，
  不包含雷达；RealSense 驱动负责相机内部 TF。
- `g1_body_tf_node` / `g1_body_tf.launch.py`：只读观察 native 状态 SHM，按腰部
  关节反馈和 pelvis IMU 姿态发布动态 TF，相机安装外参为静态 TF。

本包不提供底层 driver、`control_runtime`、RL 策略模型或 Nav2。

## 快速开始：仅桥接 cmd_vel

以下流程在 `spacemit_robot` 根目录执行。

准备 ROS2 Humble，并先构建 `application/native/humanoid_common`，确保
`robot_base`、`transport_executor` 和 `operator_client` 已安装到 `output/staging`。
G1 target 需要启用 `middleware/ros2/control/humanoid`，编译并加载环境：

```bash
source build/envsetup.sh
lunch k3-com260-kit-humanoid-unitree-g1
m -C
m -R
source output/staging/setup.bash  # zsh 使用 setup.zsh
```

终端 1 启动仿真，用 TUI 选择行走策略并手动进入 `RL`，然后按 `U` 释放控制权，
保留 TUI 和仿真进程：

```bash
run_g1.sh --sim
```

终端 2 加载同一环境并启动速度桥接：

```bash
source output/staging/setup.bash
ros2 launch humanoid g1_cmd_vel_bridge.launch.py
# 也可以直接运行：ros2 run humanoid g1_cmd_vel_bridge_node
```

终端 3 发送机器人坐标系中的纵向、横向和转向速度（单位 m/s、m/s、rad/s）：

```bash
source output/staging/setup.bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}'
```

桥接节点只使用 `operator_client` 的连接、状态查询、控制权租约和速度接口，
不选择策略、不请求状态切换、不确认故障、不触发交互动作，也不调用全局 `Stop()`。
速度限幅由 operator 服务依据当前策略的速度范围执行。

- 仅在反馈在线、HMI 已连接、状态为 `RL` 且没有故障时接收速度；其他状态收到的
  命令直接丢弃，进入 RL 后必须重新发布。桥接不检查固定策略名称。
- 收到新命令后才申请控制权；其他客户端持有控制权时丢弃命令，不抢占。
  从 TUI 交接时先按 `U`。ROS 停止发速度后，默认 0.2 秒开始下发零速度，但继续
  续租；距离最后一条有效 `cmd_vel` 达到 5 秒后才释放控制权，随后可在 TUI 按 `L`
  重新接管。5 秒内收到新速度会直接恢复转发，并重新计算两个超时，无需重新申请控制权。
  持续发布速度会持续申请／续租，交回 TUI 前先停掉发布者。
- `cmd_vel_timeout_s` 控制清零超时，`control_release_timeout_s` 控制释放超时，
  两者均从最后一条有效 `cmd_vel` 开始计时，释放超时必须大于清零超时。
- launch 默认每秒发送 10 次，单次速度有效期 600 ms；非法数值、退出 RL、故障、
  断线及发送失败都会清除缓存。退出节点时仅清零自身速度并释放自身控制权，
  不请求关机或切换状态。断线后每秒重试连接，重连后等待新的 ROS 命令。
- 订阅 `geometry_msgs/msg/Twist`，只取 `linear.x/y` 和 `angular.z`；QoS 为
  best effort、volatile、深度 1，兼容可靠和 best effort 发布者。

查看当前门控状态：

```bash
ros2 topic echo /g1_cmd_vel_bridge/status
```

全部参数及默认值定义在 [`launch/g1_cmd_vel_bridge.launch.py`](launch/g1_cmd_vel_bridge.launch.py)，
可通过同名 launch 参数覆盖：`cmd_vel_topic`、`cmd_vel_timeout_s`、`control_release_timeout_s`、`publish_rate_hz`、
`velocity_ttl_ms`、`socket_timeout_ms`、`connection_file`、`robot_config_path`，例如：

```bash
ros2 launch humanoid g1_cmd_vel_bridge.launch.py cmd_vel_topic:=/navigation/cmd_vel
```

默认读取 `$XDG_STATE_HOME/humanoid-operator/g1/connection.json`，未设置时使用
`$HOME/.local/state/humanoid-operator/g1/connection.json`。如设置了 `G1_CONFIG` 或
自定义 `operator_service.connection_file`，用 `robot_config_path:=/绝对路径/g1.yaml`
或 `connection_file:=/绝对路径/connection.json` 指向相同配置；显式 connection 文件优先。
节点和 `hmi_runtime` 应以同一用户运行，也可用于 `run_g1.sh --real` 启动的服务。

## G1 实机：腰部与 IMU 动态 TF

`g1_body_tf_node` 使用 `humanoid_common` 的 `RobotStatePacket` 协议，旁路只读
映射 `/hmrs_<transport.shm.prefix>_state`（默认 `/hmrs_g1_state`）。它不修改
共享内存读指针、不调用 transport 的消费者接口，不发送控制命令；可与
`control_runtime` 同时运行。节点仅支持同机 `transport.type: shm`。

默认发布以下链路，不含雷达和四肢 TF：

```text
base_footprint
└── base_link                 # 动态：IMU 去除 yaw 的姿态；高度为配置值
    └── pelvis                # 静态：与 base_link 重合
        └── waist_yaw_link    # 动态：waist_yaw_joint，绕 Z
            └── waist_roll_link   # 动态：waist_roll_joint，绕 X
                └── torso_link    # 动态：waist_pitch_joint，绕 Y
                    └── d435_link     # 静态：模型安装外参，可覆盖
                        └── camera_link   # 静态：单位变换
                            └── 相机 stream / optical / IMU frame（RealSense 驱动发布）
```

节点从实际运行的 `g1.yaml` 中按关节名查找腰部角度，使用经过驱动极性、零偏和
机构映射后的反馈，不固定假设数组下标。关节原点与下节静态方案一样，采用
`g1_29dof_rev_1_0.urdf`；腰部链按 yaw → roll → pitch 逐级计算，包含 roll
关节的 `[-0.0039635, 0, 0.044] m` 原点偏移。

先启动原生实机 driver，再单独启动 TF；已有 driver 时直接运行第二条命令。
TF 节点本身不启动 driver、control 或 HMI，不申请控制权。

```bash
# 终端 1：按现有实机流程启动 driver
run_driver_g1.sh --real

# 终端 2：编译并加载环境后运行，配置须与 driver 一致
source output/staging/setup.bash
ros2 launch humanoid g1_body_tf.launch.py \
  robot_config_path:=$PWD/application/native/humanoid_unitree_g1/config/g1.yaml \
  base_height:=0.793
```

也可以直接启动节点：

```bash
ros2 run humanoid g1_body_tf_node \
  "$PWD/application/native/humanoid_unitree_g1/config/g1.yaml" \
  --ros-args -p base_height:=0.793 -p publish_rate_hz:=50.0
```

相机驱动的启动方法见下一节。动态方案与 `g1_fact_tf.launch.py` 二选一，且不要
同时运行其他发布同一机体链的 `robot_state_publisher` 或仿真驱动。

### 可测量量与边界

- 当前实机 `whole_body_adapter.cpp` 将 `base_pos` 和基座线速度置零；它们不是
  里程计数据。节点不会用它们发布 `odom -> base_footprint`，也不发布
  `map -> odom`，这两段由外部视觉里程计／定位系统提供。
- `base_quat` 来自 G1 pelvis 上的 HI91 IMU，驱动已完成安装轴变换。节点对
  四元数归一化并去除 yaw，发布 `base_footprint -> base_link` 的身体倾斜。
  footprint 的世界朝向由外部里程计负责；不会把 IMU yaw 再叠加到这条边上。
- `base_height` 仍为配置常量，默认 `0.793 m` 取自模型初始高度。应按站姿实测；
  节点不能测出下蹲、腾空等高度变化。若状态估计器已提供完整基座 TF，使用
  `publish_footprint_tf:=false`，节点只提供 pelvis 与腰部／相机链。
- 节点默认以最多 `50 Hz` 读取最新状态，仅在包序号和设备时间推进且数据合法、
  无活动故障时发布动态 TF。启动先建立设备时间基线，等后续反馈再发布，避免
  使用遗留 SHM 数据。设备时间停止推进或缺少有效反馈超过 `state_timeout_s`
  （默认 `0.1 s`）时重连 SHM，反馈恢复后继续；不会持续给旧姿态打新时间戳。
- HI91 的 `packet.time` 是设备运行时间，不是 ROS 时间。TF 使用收到反馈时的
  ROS 系统时间，要求 `use_sim_time=false`；这是接收时间近似，不是相机与 IMU
  的硬件同步。RealSense 与 TF 节点应使用一致的主机时钟。

### 参数与检查

launch 支持 `robot_config_path`、`sdk_root`、`publish_rate_hz`、`state_timeout_s`、
`base_height`、`publish_footprint_tf`、`footprint_frame`、`base_frame`、`camera_frame`
以及 `camera_x/y/z`、`camera_roll/pitch/yaw`。坐标系名称应互不相同；`base_frame`
默认 `base_link`，节点另行发布它与模型 `pelvis` 的单位变换。

```bash
ros2 launch humanoid g1_body_tf.launch.py --show-args
ros2 run tf2_ros tf2_echo pelvis torso_link
ros2 run tf2_ros tf2_echo base_footprint camera_link
```

## G1 实机：D435i 静态 TF（固定姿态）

`g1_fact_tf.launch.py` 参考 `linglong_fact_tf.launch.py`，仅发布 G1 机体到
D435i 的安装链，不包含雷达。采用
`application/native/humanoid_unitree_g1/resources/urdf/g1_29dof_rev_1_0.urdf`
的关节原点和相机安装参数：

```text
base_footprint
└── base_link                  # 与 pelvis 重合，默认距地面 0.793 m
    └── pelvis
        └── waist_yaw_link
            └── waist_roll_link
                └── torso_link
                    └── d435_link
                        └── camera_link
                            └── 相机各 stream / optical / IMU frame（RealSense 驱动发布）
```

这里的机体和腰部 TF 是**固定姿态近似**，适用于腰部固定、机体近似保持该姿态的
场景；不会读取实机关节反馈。`base_height=0.793` 来自 MJCF 中 pelvis 的初始
高度，并非实机站姿标定值。应按当前站姿测量 pelvis 离地高度；机体倾斜、下蹲和
腰部运动时应改用状态估计及动态关节 TF。该 launch 不发布 `map -> odom` 或
`odom -> base_footprint`，这两段由定位和里程计节点提供。

编译并加载本包后，在两个终端分别启动 TF 和 RealSense 驱动：

```bash
ros2 launch humanoid g1_fact_tf.launch.py base_height:=0.793

# 需安装 realsense2_camera；camera_name=camera 对应 camera_link。
# camera_namespace 只影响话题命名；以下话题位于 /camera/camera/ 下。
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera publish_tf:=true \
  enable_color:=true enable_depth:=true align_depth.enable:=true \
  enable_gyro:=true enable_accel:=true
```

`torso_link -> d435_link` 默认 `xyz=[0.0576235, 0.01753, 0.42987] m`，
`rpy=[0, 0.8307767239493009, 0] rad`，即约向下俯视 47.6°。这取自模型中的
`d435_joint`，用于 D435i 的安装参考；实际安装与模型有差异时，用 `camera_x/y/z`
和 `camera_roll/pitch/yaw` 覆盖标定外参。旧版非 `rev_1_0` 模型的腰部和相机
偏移不同，不能只替换相机高度就混用整条链。

`d435_link -> camera_link` 为单位变换。RealSense 驱动负责从 `camera_link`
到彩色、深度、光学和 D435i 内置 IMU 的标定 TF；不要再手工添加一套 optical
旋转或把相机 IMU 当作 pelvis IMU。若驱动的实际根 frame 不同，通过
`camera_frame:=实际根frame` 对齐。

已有状态估计器时，设置 `publish_footprint_tf:=false`；已有
`robot_state_publisher` 发布 pelvis 到 torso 的动态关节链时，设置
`publish_body_tf:=false`，并由现有系统连接 `base_link` 与 `pelvis`。如果它也
发布模型中的 `d435_joint`，同时设置 `publish_camera_mount_tf:=false`，避免
重复 TF。无雷达的机器人描述也应删除原始 URDF 中的 `mid360_joint/link`。

检查 TF 和参数：

```bash
ros2 launch humanoid g1_fact_tf.launch.py --show-args
ros2 run tf2_ros tf2_echo base_footprint camera_link
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

默认零腰角下，`base_footprint -> camera_link` 的位置为
`[0.05366, 0.01753, 1.26687] m`，俯仰为 `0.8307767239493009 rad`。

## 常见问题

- 状态显示 `control=offline`：确认 driver、`control_runtime` 和 `hmi_runtime`
  已启动，且使用同一机器人配置。
- 收到 `/cmd_vel` 但机器人不动：确认反馈在线、HMI 已连接、状态为 `RL` 且无故障，
  并确认其他客户端未持有控制权。节点不会自动选择策略或切换 FSM；从 TUI 交接时
  先按 `U`，进入 RL 后重新发布速度。
- ROS 停止发速度后 TUI 暂时无法接管：默认 0.2 秒后清零速度，5 秒后释放控制权；
  释放后可在 TUI 按 `L` 接管，交接前先停止 ROS 速度发布者。
- 相机 TF 不连通：确认 RealSense 的实际根 frame 与 `camera_frame` 一致，且驱动
  启用了 `publish_tf`。本 launch 不提供定位和里程计 TF。
- 找不到 `robot_base` 或 `transport_executor`：先执行 `m -C` 构建 native underlay。

## 版本与发布

当前包版本和依赖信息以 `package.xml` 为准，主要兼容 ROS2 Humble。

## 贡献方式

请从最新主分支创建功能或修复分支，确保构建、单元测试和 lint 检查通过后提交
Pull Request。贡献者与维护者名单见：`CONTRIBUTORS.md`。

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
