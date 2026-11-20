#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    # ================== TF 静态变换 ==================
    tf2_node_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.0', '0.0', '0.0001', '0.0', '0.0', '0.0',
                   'base_footprint', 'base_link'],
    )

    # Hesai 的 Y 轴向前，ROS 的 X 轴向前；绕 Z 轴旋转 -90° 对齐雷达坐标系。
    tf2_node_3d_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_base_to_hesai_lidar',
        arguments=['0.01', '0.0', '0.25', '-1.57079632679', '0.0', '0.0',
                   'base_link', 'hesai_lidar'],
    )

    tf2_node_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_base_to_laser',
        arguments=['-0.035', '0.0', '0.30', '3.14159', '0.0', '0.0',
                   'base_link', 'laser_link'],
    )

    tf2_node_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_base_to_imu',
        arguments=['0.0', '0.0', '0.01', '0', '0.0', '0.0',
                   'base_link', 'imu_link'],
    )

    tf2_node_rgbd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_rgbd',
        arguments=['0.035', '0.039', '1.56', '0.0', '0.523', '0.0',
                   'base_link', 'camera_link'],
    )
    # realsense 左侧红外为camera_link
    # D455 左侧第一个相机为红外（从后往前看）

    tf2_node_rplidar_link = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_pub_base_to_rplidar_link',
            arguments=['0.0', '0.0', '1.56', '0.0', '0.0', '0.0',
                       'base_link', 'rplidar_link'],
        )

    return LaunchDescription([
        tf2_node_base,
        tf2_node_3d_laser,
        tf2_node_laser,
        tf2_node_imu,
        tf2_node_rgbd,
        tf2_node_rplidar_link,
    ])
