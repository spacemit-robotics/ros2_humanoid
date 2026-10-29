# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    parameters = [
        ('cmd_vel_topic', '/cmd_vel', str),
        # Zero velocity after this interval without a new cmd_vel; retain the lease.
        ('cmd_vel_timeout_s', '0.2', float),
        # Release control after this interval since the same last cmd_vel.
        ('control_release_timeout_s', '5.0', float),
        ('publish_rate_hz', '10.0', float),
        ('velocity_ttl_ms', '600', int),
        ('socket_timeout_ms', '600', int),
        # Empty: use the local G1 connection.json written by hmi_runtime.
        ('connection_file', '', str),
        # Optional robot configuration to locate its operator connection file.
        ('robot_config_path', '', str),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default)
        for name, default, _ in parameters
    ] + [
        Node(
            package='humanoid',
            executable='g1_cmd_vel_bridge_node',
            name='g1_cmd_vel_bridge',
            output='screen',
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                for name, _, value_type in parameters
            }],
        ),
    ])
