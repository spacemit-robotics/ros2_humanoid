# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""G1 hardware waist and pelvis attitude TF from a read-only native SHM tap."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_sdk_root():
    root = os.environ.get('SDK_ROOT') or os.environ.get('SROBOTIS_ROOT')
    if root:
        return root
    candidate = os.path.join(os.getcwd(), 'spacemit_robot')
    return candidate if os.path.isdir(candidate) else os.getcwd()


def generate_launch_description():
    parameters = [
        ('publish_rate_hz', '50.0', float),
        ('state_timeout_s', '0.1', float),
        # Hardware reports attitude, not floating-base position or ground height.
        ('base_height', '0.793', float),
        ('publish_footprint_tf', 'true', bool),
        ('footprint_frame', 'base_footprint', str),
        ('base_frame', 'base_link', str),
        ('camera_frame', 'camera_link', str),
        ('camera_x', '0.0576235', float),
        ('camera_y', '0.01753', float),
        ('camera_z', '0.42987', float),
        ('camera_roll', '0.0', float),
        ('camera_pitch', '0.8307767239493009', float),
        ('camera_yaw', '0.0', float),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('sdk_root', default_value=_default_sdk_root()),
        DeclareLaunchArgument('robot_config_path', default_value=[
            LaunchConfiguration('sdk_root'),
            '/application/native/humanoid_unitree_g1/config/g1.yaml',
        ]),
        *[DeclareLaunchArgument(name, default_value=default)
          for name, default, _ in parameters],
        Node(
            package='humanoid', executable='g1_body_tf_node', name='g1_body_tf',
            output='screen', arguments=[LaunchConfiguration('robot_config_path')],
            parameters=[{
                'use_sim_time': False,
                **{name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                   for name, _, kind in parameters},
            }],
        ),
    ])
