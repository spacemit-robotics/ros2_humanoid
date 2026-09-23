# Example:
#
#   Bringup turtlebot3:
#     $ export TURTLEBOT3_MODEL=waffle
#     $ export LDS_MODEL=LDS-01
#     $ ros2 launch turtlebot3_bringup robot.launch.py
#
#   SLAM:
#     $ ros2 launch rtabmap_demos turtlebot3_rgbd.launch.py
#
#   Navigation (install nav2_bringup package):
#     $ ros2 launch nav2_bringup navigation_launch.py
#     $ ros2 launch nav2_bringup rviz_launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    launch_point_cloud_xyz = LaunchConfiguration('launch_point_cloud_xyz')
    min_depth = LaunchConfiguration('min_depth')
    max_depth = LaunchConfiguration('max_depth')
    grid_range_min = LaunchConfiguration('grid_range_min')
    grid_range_max = LaunchConfiguration('grid_range_max')

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'Reg/Force3DoF':'true',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          'Grid/RangeMin':ParameterValue(grid_range_min, value_type=str),
          'Grid/RangeMax':ParameterValue(grid_range_max, value_type=str),
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/MaxGroundHeight':'0.1', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'1.63',  # All points over 1 meter are ignored
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    remappings=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_rect_raw')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='设为 true 时使用仿真（Gazebo）时钟'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='是否以定位模式启动'),

        DeclareLaunchArgument(
            'launch_point_cloud_xyz', default_value='true',
            description=(
                '是否启动 point_cloud_xyz；当其他 launch 文件已经发布 '
                '/camera/cloud 时应设为 false')),

        DeclareLaunchArgument(
            'min_depth', default_value='0.1',
            description=(
                '转换为 /camera/cloud 的最小深度，单位为米；'
                '设为 0 时禁用最小深度过滤')),

        DeclareLaunchArgument(
            'max_depth', default_value='6.0',
            description=(
                '转换为 /camera/cloud 的最大深度，单位为米；'
                '设为 0 时禁用最大深度过滤')),

        DeclareLaunchArgument(
            'grid_range_min', default_value='0.1',
            description=(
                'RTAB-Map 栅格地图和障碍物处理使用的最小点云距离，'
                '单位为米')),

        DeclareLaunchArgument(
            'grid_range_max', default_value='6.0',
            description=(
                'RTAB-Map 栅格地图和障碍物处理使用的最大点云距离，'
                '单位为米')),

        # Nodes to launch

        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)

        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[parameters],
        #     remappings=remappings),

        # Obstacle detection with the camera for nav2 local costmap.
        # First, we need to convert depth image to a point cloud.
        # Second, we segment the floor from the obstacles.
        Node(
            condition=IfCondition(launch_point_cloud_xyz),
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'decimation': 2,
                         'min_depth': ParameterValue(
                             min_depth, value_type=float),
                         'max_depth': ParameterValue(
                             max_depth, value_type=float),
                         'voxel_size': 0.02}],
            remappings=[('depth/image', '/camera/depth/image_rect_raw'),
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        ('cloud', '/camera/cloud')]),
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[parameters],
            remappings=[('cloud', '/camera/cloud'),
                        ('obstacles', '/camera/obstacles'),
                        ('ground', '/camera/ground')]),
    ])
