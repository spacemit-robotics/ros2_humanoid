# linglong slam 与 nav2

## 依赖

```bash
sudo apt install python3-zmq python3-serial python3-transforms3d \
libspdlog-dev libconsole-bridge-dev liborocos-kdl-dev nlohmann-json3-dev liblgpio-dev \
liblttng-ust-dev libgpiod-dev ros-humble-camera-info-manager ros-humble-slam-toolbox \
ros-humble-cartographer ros-humble-cartographer-ros ros-humble-nav2* 'ros-humble-rtabmap*' \
ros-humble-aruco-markers-msgs ros-humble-realsense2-camera ros-dev-tools ros-humble-desktop \
udev git cmake ninja-build build-essential patchelf libboost-serialization-dev libssl-dev libsuitesparse-dev \
libglew-dev libepoxy-dev libx11-dev libwayland-dev libjpeg-dev libpng-dev libtiff-dev wget \
libeigen3-dev opencv-spacemit=4.14.0-2bb4
```

```bash
cd ~
wget https://archive.spacemit.com/ros2/prebuilt_libs/source_code_common/librealsense-2.57.4.tar.gz
tar xzvf librealsense-2.57.4.tar.gz
cd librealsense-2.57.4/
./scripts/setup_udev_rules.sh
cd ..
rm -rf librealsense-2.57.4 librealsense-2.57.4.tar.gz
```

## realsense

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=/ \
  enable_color:=true enable_depth:=true \
  rgb_camera.color_profile:=640,480,15 \
  depth_module.depth_profile:=640,480,15 \
  align_depth.enable:=true enable_sync:=true
```

## 启动基础 TF

```bash
ros2 launch humanoid linglong_fact_tf.launch.py
```

## 里程计

```bash
ros2 launch peripherals_lidar_node lidar_2d.launch.py channel_type:=serial model:=RPLIDAR serial_baudrate:=460800 \
flip_x_axis:=true frame_id:=rplidar_link serial_port:=/dev/ttyUSB1
```

```bash
ros2 launch humanoid linglong_lidar_odom.launch.py
```

## rgbd slam

```bash
ros2 launch humanoid linglong_rgbd_slam.launch.py
```

## nav2

```bash
ros2 launch humanoid linglong_rgbd_nav2.launch.py
```

## 可视化

```bash
source ~/visual_ws/install/setup.bash
ros2 launch visualization display_rgbd.launch.py
```
