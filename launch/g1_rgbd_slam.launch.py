# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    localization = LaunchConfiguration('localization')
    use_imu = LaunchConfiguration('use_imu')
    imu_raw_topic = LaunchConfiguration('imu_raw_topic')
    imu_filtered_topic = LaunchConfiguration('imu_filtered_topic')

    # 公共参数：SLAM、定位和可视化节点共用。
    parameters = {
        # 坐标系、时间和输入数据。
        'use_sim_time': sim_time,
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'subscribe_depth': True,
        'subscribe_odom_info': False,
        'approx_sync': False,
        'queue_size': 30,
        'publish_tf': True,
        'use_action_for_goal': True,
        'database_path': ParameterValue(LaunchConfiguration('database_path'), value_type=str),

        # 建图频率和平面运动约束。
        'Rtabmap/DetectionRate': '5.0',
        'Reg/Force3DoF': 'true',

        # 从深度图生成二维栅格地图，并按高度区分地面和障碍物。
        'Grid/FromDepth': 'true',
        'Grid/RayTracing': 'true',
        'Grid/3D': 'false',
        'Grid/RangeMax': '6.0',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MinGroundHeight': '-0.20',
        'Grid/MaxGroundHeight': '0.12',
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/CellSize': '0.05',

        # 关闭 IMU 重力约束，以 16 位格式保存深度图。
        'Optimizer/GravitySigma': '0',
        'Mem/SaveDepth16Format': 'true',
    }

    # SLAM/定位专用参数：从 TF 获取外部里程计，并使用滤波后的 IMU 姿态。
    slam_parameters = {
        **parameters,
        # 方差越小，优化时越信任外部 LIO；分别使用 m^2 和 rad^2。
        'odom_tf_linear_variance': ParameterValue(
            LaunchConfiguration('odom_tf_linear_variance'), value_type=float),
        'odom_tf_angular_variance': ParameterValue(
            LaunchConfiguration('odom_tf_angular_variance'), value_type=float),
        'Optimizer/VarianceIgnored': 'false',

        # IMU 需要有效姿态及到 base_footprint 的 TF。
        'qos_imu': 2,  # Best effort，兼容相机常用的 IMU 发布 QoS。
        'Optimizer/GravitySigma': ParameterValue(
            PythonExpression([
                LaunchConfiguration('imu_gravity_sigma'),
                " if '", use_imu, "'.lower() in ('true', '1') else 0",
            ]),
            value_type=str,
        ),
    }

    # 公共话题映射：使用外部里程计和对齐到彩色图的深度图。
    remappings = [
        ('odom', '/odom'),
        ('rgb/image', '/camera/color/image_raw'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('grid_map', '/map'),
    ]

    # RTAB-Map 固定创建 IMU 订阅；关闭时映射到隔离话题，不接收相机 IMU。
    slam_remappings = remappings + [
        ('imu', PythonExpression([
            "'", imu_filtered_topic, "' if '", use_imu,
            "'.lower() in ('true', '1') else '/rtabmap/imu_disabled'",
        ])),
    ]

    return LaunchDescription([
        # 启动参数。
        DeclareLaunchArgument(
            'use_sim_time', default_value='false'),

        DeclareLaunchArgument(
            'localization', default_value='false'),

        DeclareLaunchArgument(
            'odom_tf_linear_variance', default_value='0.0001',
            description='TF odometry translation variance in m^2; smaller values increase trust.'),

        DeclareLaunchArgument(
            'odom_tf_angular_variance', default_value='0.0001',
            description='TF odometry rotation variance in rad^2; smaller values increase trust.'),

        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='Filter raw IMU data and use its orientation for gravity constraints.'),

        DeclareLaunchArgument(
            'imu_raw_topic', default_value='/camera/imu',
            description='Raw IMU topic with angular velocity and linear acceleration.'),

        DeclareLaunchArgument(
            'imu_filtered_topic', default_value='/camera/imu_filtered',
            description='Filtered IMU topic with orientation, consumed by RTAB-Map.'),

        DeclareLaunchArgument(
            'imu_gain_acc', default_value='0.01',
            description='Complementary filter accelerometer correction gain.'),

        DeclareLaunchArgument(
            'imu_gravity_sigma', default_value='0.3',
            description='RTAB-Map IMU gravity constraint sigma; 0 disables constraints.'),

        DeclareLaunchArgument(
            'start_viz', default_value='false'),

        DeclareLaunchArgument(
            'database_path', default_value=os.path.expanduser('~/.ros/g1_rgbd_map.db')),

        # 先融合原始加速度和角速度，再向 SLAM/定位节点提供有效姿态。
        Node(
            condition=IfCondition(use_imu),
            package='imu_complementary_filter', executable='complementary_filter_node',
            name='g1_imu_filter', output='screen',
            parameters=[{
                'use_sim_time': sim_time,
                'use_mag': False,
                'publish_tf': False,
                'do_adaptive_gain': True,
                'do_bias_estimation': True,
                'gain_acc': ParameterValue(
                    LaunchConfiguration('imu_gain_acc'), value_type=float),
                # QoS 参数使用重映射后的完整话题名，兼容 RealSense 的 Best effort。
                ('qos_overrides.', imu_raw_topic, '.subscription.reliability'): 'best_effort',
            }],
            remappings=[
                ('imu/data_raw', imu_raw_topic),
                ('imu/data', imu_filtered_topic),
            ],
        ),

        # SLAM 模式：启动时清空指定数据库，重新建图。
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap',
            name='rtabmap', output='screen',
            parameters=[slam_parameters, {'Mem/IncrementalMemory': 'true'}],
            remappings=slam_remappings,
            arguments=['-d'],
        ),

        # 定位模式：加载数据库中的全部节点，不再增量建图。
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap',
            name='rtabmap', output='screen',
            parameters=[slam_parameters, {
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
            }],
            remappings=slam_remappings,
        ),

        # 可视化：通过 start_viz 参数选择是否启动。
        Node(
            condition=IfCondition(LaunchConfiguration('start_viz')),
            package='rtabmap_viz', executable='rtabmap_viz',
            name='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=remappings,
        ),

        # 障碍物检测第一步：将深度图转换为点云。
        Node(
            package='rtabmap_util', executable='point_cloud_xyz',
            name='g1_rgbd_point_cloud', output='screen',
            parameters=[{
                'use_sim_time': sim_time,
                'decimation': 2,
                'max_depth': 6.0,
                'voxel_size': 0.04,
            }],
            remappings=[
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                ('depth/camera_info', '/camera/aligned_depth_to_color/camera_info'),
                ('cloud', '/camera/cloud'),
            ],
        ),

        # 障碍物检测第二步：从点云中分离地面和障碍物，供局部代价地图使用。
        Node(
            package='rtabmap_util', executable='obstacles_detection',
            name='g1_rgbd_obstacles', output='screen',
            parameters=[{
                'use_sim_time': sim_time,
                'frame_id': 'base_footprint',
                'wait_for_transform': 0.5,
                'Grid/NormalsSegmentation': 'false',
                'Grid/MinGroundHeight': '-0.20',
                'Grid/MaxGroundHeight': '0.12',
                'Grid/MaxObstacleHeight': '2.0',
                'Grid/ClusterRadius': '0.10',
                'Grid/MinClusterSize': '10',
            }],
            remappings=[
                ('cloud', '/camera/cloud'),
                ('ground', '/camera/ground'),
                ('obstacles', '/camera/obstacles'),
            ],
        ),
    ])
