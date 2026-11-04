# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package = get_package_share_directory('humanoid')
    navigation = os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                              'navigation_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params', default_value=os.path.join(
            package, 'config', 'g1_rgbd_nav2_params.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation), launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params'),
                'autostart': 'true', 'use_composition': 'False',
            }.items(),
        ),
    ])
