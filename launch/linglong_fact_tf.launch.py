#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    # hesai是Y轴向前，X轴向左，Z轴向上的坐标系，而ROS通常使用X轴向前，Y轴向左，Z轴向上的坐标系，因此需要进行坐标变换调整。这里的旋转角度 -1.57079632679（-90度）将 hesai_lidar 的坐标系从 ROS 标准坐标系旋转到正确的方向，使得激光雷达的数据能够正确地与 ROS 中的其他传感器数据进行融合和处理。
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
