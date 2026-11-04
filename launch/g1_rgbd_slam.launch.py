# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    localization = LaunchConfiguration('localization')
    parameters = {
        'use_sim_time': sim_time, 'frame_id': 'base_footprint', 'odom_frame_id': 'odom',
        'map_frame_id': 'map', 'subscribe_depth': True, 'subscribe_odom_info': False,
        'approx_sync': False, 'queue_size': 30, 'publish_tf': True, 'use_action_for_goal': True,
        'database_path': ParameterValue(LaunchConfiguration('database_path'), value_type=str),
        'Rtabmap/DetectionRate': '5.0', 'Reg/Force3DoF': 'true',
        'Grid/FromDepth': 'true', 'Grid/RayTracing': 'true', 'Grid/3D': 'false',
        'Grid/RangeMax': '6.0', 'Grid/NormalsSegmentation': 'false',
        'Grid/MinGroundHeight': '-0.20', 'Grid/MaxGroundHeight': '0.12',
        'Grid/MaxObstacleHeight': '2.0', 'Grid/CellSize': '0.05',
        'Optimizer/GravitySigma': '0',
        'Mem/SaveDepth16Format': 'true',
    }
    remappings = [
        ('odom', '/odom'), ('rgb/image', '/camera/color/image_raw'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'), ('rgb/camera_info', '/camera/color/camera_info'),
        ('grid_map', '/map'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('start_viz', default_value='false'),
        DeclareLaunchArgument(
            'database_path', default_value=os.path.expanduser('~/.ros/g1_rgbd_textured_map.db')),
        Node(
            condition=UnlessCondition(localization), package='rtabmap_slam', executable='rtabmap',
            name='rtabmap', output='screen',
            parameters=[parameters, {'Mem/IncrementalMemory': 'true'}],
            remappings=remappings,
        ),
        Node(
            condition=IfCondition(localization), package='rtabmap_slam', executable='rtabmap',
            name='rtabmap', output='screen', parameters=[parameters, {
                'Mem/IncrementalMemory': 'false', 'Mem/InitWMWithAllNodes': 'true',
            }], remappings=remappings,
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('start_viz')),
            package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz', output='screen',
            parameters=[parameters], remappings=remappings,
        ),
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', name='g1_rgbd_point_cloud',
            output='screen', parameters=[{
                'use_sim_time': sim_time, 'decimation': 2, 'max_depth': 6.0, 'voxel_size': 0.04,
            }], remappings=[
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                ('depth/camera_info', '/camera/aligned_depth_to_color/camera_info'), ('cloud', '/camera/cloud'),
            ],
        ),
        Node(
            package='rtabmap_util', executable='obstacles_detection', name='g1_rgbd_obstacles',
            output='screen', parameters=[{
                'use_sim_time': sim_time, 'frame_id': 'base_footprint',
                'wait_for_transform': 0.5,
                'Grid/NormalsSegmentation': 'false', 'Grid/MinGroundHeight': '-0.20',
                'Grid/MaxGroundHeight': '0.12', 'Grid/MaxObstacleHeight': '2.0',
                'Grid/ClusterRadius': '0.10', 'Grid/MinClusterSize': '10',
            }], remappings=[
                ('cloud', '/camera/cloud'), ('ground', '/camera/ground'),
                ('obstacles', '/camera/obstacles'),
            ],
        ),
    ])
