# Unitree G1 人形机器人 ROS2 控制

## 项目简介

`humanoid` 将 ROS2 `/cmd_vel` 接入 G1 的 native 运控链路，并提供实机腰部、
IMU 姿态动态 TF、D435i 安装 TF，以及 RGBD 建图和导航启动配置。
LingLong 的导航桥接、语音动作和头部 TF 说明见
[README_linglong.md](README_linglong.md)。

- `g1_cmd_vel_bridge_node`：通过公开的 `operator_client` 接口，在已有 RL 状态下
  申请控制权并转发速度，不切换机器人状态或策略。
- `g1_fact_tf.launch.py`：按 G1 `rev_1_0` 模型发布固定机体姿态和 D435i 安装 TF，
  不包含雷达；RealSense 驱动负责相机内部 TF。
- `g1_body_tf_node` / `g1_body_tf.launch.py`：只读观察 native 状态 SHM，按腰部
  关节反馈和 pelvis IMU 姿态发布动态 TF，相机安装外参为静态 TF。
- `g1_rgbd_slam.launch.py`：启动 RTAB-Map 建图／定位、深度点云及地面／障碍物分割。
- `g1_rgbd_nav2.launch.py`：使用 `config/g1_rgbd_nav2_params.yaml` 启动 Nav2 导航节点。

底层 driver、`control_runtime` 和 RL 策略模型由 native 工程提供；RTAB-Map、Nav2
及 RealSense 驱动需另行安装。本包的 RGBD launch 不启动相机、里程计或 native 运控进程。
完整 MuJoCo RGBD 仿真流程见
[humanoid_mujoco/README_g1.md](../../simulation/humanoid_mujoco/README_g1.md)。

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

- 仅在反馈在线、HMI 已连接、状态为 `RL`、未启用轨迹播放且没有活动或锁存故障时
  接收速度；其他状态收到的命令直接丢弃，进入 RL 后必须重新发布。
  桥接不检查固定策略名称。
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
  直接 `ros2 run` 时，`velocity_ttl_ms` 和 `socket_timeout_ms` 的节点默认值均为
  300 ms；launch 将这两项覆盖为 600 ms。
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

## RGBD 建图与 Nav2 导航

### 依赖与输入

除前述 native underlay 与本包外，运行环境还需 `rtabmap_slam`、`rtabmap_util`、
`nav2_bringup`、`nav2_navfn_planner` 和 `nav2_regulated_pure_pursuit_controller`；
启用可视化时还需 `rtabmap_viz`。完整运行依赖见 [`package.xml`](package.xml)。

启动前需由上游提供以下话题和 TF；实机默认使用系统时间 `use_sim_time:=false`：

| 输入 | 要求 |
| --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image`，彩色图像 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image`，与彩色图像对齐的深度图 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo`，对应对齐图像的内参 |
| `/odom` | `nav_msgs/msg/Odometry`，连续的平面里程计 |
| `odom -> base_footprint` | 外部里程计提供的动态 TF |
| `base_footprint -> 相机 optical frame` | 本文机体／安装 TF 与相机驱动 TF 组成的完整链 |

当前 `g1_rgbd_slam.launch.py` 固定使用这些话题名，且 RTAB-Map 设置
`approx_sync=false`，要求输入 RGB、深度和 CameraInfo 时间戳匹配。RealSense
驱动原始话题通常与这里不同；使用上一节的驱动命令时，应在上游将以下话题重映射或转发：

| RealSense 话题（上一节默认命名） | 本方案输入 |
| --- | --- |
| `/camera/camera/color/image_raw` | `/camera/image_raw` |
| `/camera/camera/aligned_depth_to_color/image_raw` | `/camera/depth/image_raw` |
| `/camera/camera/color/camera_info` | `/camera/camera_info` |

必须使用对齐到彩色的深度图和对应内参；仅修改话题名不能完成图像对齐或时间同步。
当前 launch 未暴露话题名和同步方式参数；需要其他接口时，调整
[`g1_rgbd_slam.launch.py`](launch/g1_rgbd_slam.launch.py) 中的 remappings 与 parameters。

实机 `g1_body_tf_node` 只提供身体倾斜和腰部／相机链，**不提供 `/odom` 或
`odom -> base_footprint`**。本节 launch 也不启动 `rgbd_odometry`；需先接入并验证
外部视觉里程计或状态估计器。RTAB-Map 发布 `map -> odom`，不要同时让其他定位节点
发布这条 TF。使用 MuJoCo RGBD DRIVER 时，里程计和机体／相机 TF 已由仿真提供，
不要再启动实机 TF launch。

### 分步启动

以下命令在 `spacemit_robot` 根目录执行，各终端均需先加载
`output/staging/setup.bash`。实机先启动 native 运控、相机、机体 TF 和外部里程计，
确认上表输入就绪后，再分别启动：

```bash
# 终端 1：建图、点云和障碍物分割；按需启用 RTAB-Map 可视化
ros2 launch humanoid g1_rgbd_slam.launch.py start_viz:=true \
  database_path:=$HOME/.ros/g1_real_map.db

# 终端 2：导航（不启动建图、定位、相机或速度桥）
ros2 launch humanoid g1_rgbd_nav2.launch.py

# 终端 3：速度桥；若前面的流程已启动该节点，不要重复启动
ros2 launch humanoid g1_cmd_vel_bridge.launch.py
```

用 TUI 选择经验证的行走策略并手动进入 `RL`，按 `U` 释放控制权后，再通过 RViz
的 `Nav2 Goal` 或 `NavigateToPose` action 发送目标。导航节点不会自动选择策略或
切换 FSM。切回 TUI 前先取消导航目标并停止其他速度发布者，等待桥接释放控制权后按 `L`。

启动后的数据链为：

```text
RGBD + /odom + TF -> RTAB-Map -> /map、map -> odom
深度图 -> /camera/cloud -> /camera/ground、/camera/obstacles -> Nav2 costmap
Nav2 -> velocity_smoother -> /cmd_vel -> g1_cmd_vel_bridge -> native operator 服务
```

这套配置使用深度点云，无需 `/scan`。SLAM launch 始终启动点云与分割节点，
`start_viz` 仅控制 `rtabmap_viz`，不启动 RViz。Nav2 launch 仅包含
`nav2_bringup/navigation_launch.py`，自动激活导航节点，不启动 AMCL 或 map server。

### 参数、地图与定位

| launch | 参数与默认值 |
| --- | --- |
| `g1_rgbd_slam.launch.py` | `use_sim_time:=false`、`localization:=false`、`start_viz:=false`、`database_path:=$HOME/.ros/g1_rgbd_textured_map.db` |
| `g1_rgbd_nav2.launch.py` | `use_sim_time:=false`、`params:=<humanoid 安装目录>/config/g1_rgbd_nav2_params.yaml` |

数据库默认路径在启动时展开为当前用户的绝对路径，不会自动删除已有数据库。
新建地图时指定新的数据库路径；实机和仿真应分别保存。正常退出建图后，用同一数据库
进入定位模式，再单独启动上述 Nav2 与速度桥：

```bash
ros2 launch humanoid g1_rgbd_slam.launch.py localization:=true \
  database_path:=$HOME/.ros/g1_real_map.db
```

定位模式设置 `Mem/IncrementalMemory=false`、`Mem/InitWMWithAllNodes=true`。
也可将当前 `/map` 导出为二维栅格，但栅格文件不能替代 RTAB-Map 的视觉定位数据库：

```bash
ros2 run nav2_map_server map_saver_cli -f "$HOME/g1_map" \
  --ros-args -p use_sim_time:=false
```

仿真中使用本包的 SLAM／Nav2 launch 时，两者均传 `use_sim_time:=true`，并确保
仿真发布 `/clock`。Nav2 launch 会用该参数覆盖 YAML 中的 `use_sim_time`。
完整仿真也可直接按 `humanoid_mujoco` 的 README 启动，避免重复启动同一组节点。

导航参数集中在 [`config/g1_rgbd_nav2_params.yaml`](config/g1_rgbd_nav2_params.yaml)。
当前使用 NavFn 全局规划、Regulated Pure Pursuit 路径跟踪、点云 voxel costmap 和
速度平滑；限速参考 native `humanoid_unitree_g1/config/g1.yaml` 中三种常规运控
（`stand`、`walk_mjlab`、`walk_rlgym`）的共同范围：纵向 `[-0.4, 0.7] m/s`、
角速度 `[-1.0, 1.0] rad/s`。当前导航禁用倒退，平滑器纵向限速为 `[0.0, 0.7] m/s`，
不输出横向速度。巡航速度为 `0.6 m/s`，接近目标及
曲率／代价调速的最低速度为 `0.15 m/s`，为 MJLab 的 `0.1` 步态相位阈值留出余量。
纵向／角向加速度为 `1.0 m/s²`／`2.0 rad/s²`，减速度幅值为 `2.5 m/s²`／
`2.5 rad/s²`，与 native `operator_service.teleop` 配置一致。若使用限速更低的自定义
机器人配置（如 `g1_rgbd.yaml`），需同步下调 Nav2 限速；更换策略或平台后，仍需验证
速度范围、机体 footprint、障碍物高度与目标容差。自定义配置示例：

```bash
ros2 launch humanoid g1_rgbd_nav2.launch.py params:=/绝对路径/g1_nav2.yaml
ros2 launch humanoid g1_rgbd_slam.launch.py --show-args
ros2 launch humanoid g1_rgbd_nav2.launch.py --show-args
```

## 常见问题

- 桥接状态显示 `online=false` 或连接失败：确认 driver、`control_runtime` 和
  `hmi_runtime` 已启动，且使用同一机器人配置和 operator connection 文件。
- 收到 `/cmd_vel` 但机器人不动：确认反馈在线、HMI 已连接、状态为 `RL` 且无故障，
  未启用轨迹播放，并确认其他客户端未持有控制权。节点不会自动选择策略或切换 FSM；从 TUI 交接时
  先按 `U`，进入 RL 后重新发布速度。
- ROS 停止发速度后 TUI 暂时无法接管：默认 0.2 秒后清零速度，5 秒后释放控制权；
  释放后可在 TUI 按 `L` 接管，交接前先停止 ROS 速度发布者。
- 相机 TF 不连通：确认 RealSense 的实际根 frame 与 `camera_frame` 一致，且驱动
  启用了 `publish_tf`。本 launch 不提供定位和里程计 TF。
- RTAB-Map 没有地图或提示等待数据：检查 RGB、对齐深度、CameraInfo 的话题名与
  时间戳，以及 `/odom`、`odom -> base_footprint` 和相机 TF 是否就绪；实机不要
  启用 `use_sim_time`。默认使用精确同步，仅有话题但时间戳不匹配也无法建图。
- Nav2 未输出速度或停滞：先检查 `/map`、定位 TF、`/camera/obstacles` 和导航节点
  lifecycle 状态，再检查 `/cmd_vel` 与 `/g1_cmd_vel_bridge/status`。地图和路径正常
  但机器人不动时，还需核对策略速度阈值及控制权；Nav2 不负责进入 RL。
- 找不到 `robot_base` 或 `transport_executor`：先执行 `m -C` 构建 native underlay。

## 版本与发布

当前包版本和依赖信息以 `package.xml` 为准，主要兼容 ROS2 Humble。

## 贡献方式

请从最新主分支创建功能或修复分支，确保构建、单元测试和 lint 检查通过后提交
Pull Request。贡献者与维护者名单见：`CONTRIBUTORS.md`。

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
