import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_sdk_root():
    if os.environ.get('SDK_ROOT'):
        return os.environ['SDK_ROOT']
    cwd = os.getcwd()
    candidate = os.path.join(cwd, 'spacemit_robot')
    return candidate if os.path.isdir(candidate) else cwd


def generate_launch_description():
    sdk_root = LaunchConfiguration('sdk_root')
    robot_config_path = LaunchConfiguration('robot_config_path')
    camera_parent_frame = LaunchConfiguration('camera_parent_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    head_tf_node = Node(
        package='humanoid',
        executable='humanoid_head_tf_node',
        name='humanoid_head_tf',
        output='screen',
        arguments=[robot_config_path],
    )

    camera_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_rgbd',
        output='screen',
        # Positional order required by ROS2 Humble: x y z yaw pitch roll.
        arguments=[
            camera_x, camera_y, camera_z,
            camera_yaw, camera_pitch, camera_roll,
            camera_parent_frame, camera_frame,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('sdk_root', default_value=_default_sdk_root()),
        DeclareLaunchArgument(
            'robot_config_path',
            default_value=[
                sdk_root,
                '/application/native/humanoid_linglong/config/linglong.yaml',
            ],
        ),
        DeclareLaunchArgument('camera_parent_frame', default_value='head_pitch_link'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('camera_x', default_value='0.035'),
        DeclareLaunchArgument('camera_y', default_value='0.039'),
        DeclareLaunchArgument('camera_z', default_value='0.24'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.523'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        head_tf_node,
        camera_static_tf_node,
    ])
