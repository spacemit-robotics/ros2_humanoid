import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('humanoid')
    params_path = os.path.join(pkg_path, 'config', 'linglong_nav_rl_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_path',
            default_value=(
                'spacemit_robot/application/native/'
                'humanoid_linglong/config/linglong.yaml'),
        ),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('auto_fsm', default_value='false'),
        DeclareLaunchArgument('auto_policy', default_value='walk'),
        Node(
            package='humanoid',
            executable='humanoid_nav_rl_bridge_node',
            name='humanoid_nav_rl_bridge',
            output='screen',
            parameters=[
                params_path,
                {
                    'robot_config_path': LaunchConfiguration('robot_config_path'),
                    'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                    'auto_fsm': LaunchConfiguration('auto_fsm'),
                    'auto_policy': LaunchConfiguration('auto_policy'),
                },
            ],
        ),
    ])
