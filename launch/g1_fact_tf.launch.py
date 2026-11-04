#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""G1 hardware sensor TF for a fixed body pose and a RealSense D435i.

Geometry: humanoid_unitree_g1/resources/urdf/g1_29dof_rev_1_0.urdf.
This launch does not estimate the floating base or read waist feedback. Disable
its body/footprint publishers when a state estimator or robot_state_publisher
provides those edges. RealSense owns all TF below camera_link.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _static_tf(name, parent, child, xyz=('0', '0', '0'),
               rpy=('0', '0', '0'), condition=None):
    # Named options avoid the positional yaw/pitch/roll ordering ambiguity.
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='screen',
        condition=condition,
        arguments=[
            '--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
            '--roll', rpy[0], '--pitch', rpy[1], '--yaw', rpy[2],
            '--frame-id', parent, '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    parameters = [
        ('footprint_frame', 'base_footprint', 'Ground projection frame.'),
        ('base_frame', 'base_link', 'Body frame coincident with the model pelvis.'),
        ('camera_frame', 'camera_link', 'RealSense driver root TF frame.'),
        ('publish_footprint_tf', 'true',
         'Publish a nominal fixed ground-to-pelvis pose; disable with an estimator.'),
        ('publish_body_tf', 'true',
         'Publish pelvis and fixed waist pose; disable with robot_state_publisher.'),
        # MJCF pelvis initial height, NOT a measured hardware standing height.
        ('base_height', '0.793', 'Actual pelvis height above ground, in metres.'),
        ('base_roll', '0.0', 'Fixed body roll relative to footprint, in radians.'),
        ('base_pitch', '0.0', 'Fixed body pitch relative to footprint, in radians.'),
        ('waist_yaw', '0.0', 'Fixed waist yaw joint position, in radians.'),
        ('waist_roll', '0.0', 'Fixed waist roll joint position, in radians.'),
        ('waist_pitch', '0.0', 'Fixed waist pitch joint position, in radians.'),
        # d435_joint: measured corrections can override the model defaults.
        ('camera_x', '0.0576235', 'D435i mount X relative to torso_link, in metres.'),
        ('camera_y', '0.01753', 'D435i mount Y relative to torso_link, in metres.'),
        ('camera_z', '0.42987', 'D435i mount Z relative to torso_link, in metres.'),
        ('camera_roll', '0.0', 'D435i mount roll, in radians.'),
        ('camera_pitch', '0.8307767239493009', 'D435i mount pitch, in radians.'),
        ('camera_yaw', '0.0', 'D435i mount yaw, in radians.'),
        ('publish_camera_mount_tf', 'true',
         'Disable if another publisher already provides torso_link -> d435_link.'),
    ]
    config = LaunchConfiguration
    body_condition = IfCondition(config('publish_body_tf'))
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in parameters],
        _static_tf(
            'g1_tf_footprint_to_base', config('footprint_frame'), config('base_frame'),
            xyz=('0', '0', config('base_height')),
            rpy=(config('base_roll'), config('base_pitch'), '0'),
            condition=IfCondition(config('publish_footprint_tf')),
        ),
        _static_tf('g1_tf_base_to_pelvis', config('base_frame'), 'pelvis',
                   condition=body_condition),
        _static_tf('g1_tf_waist_yaw', 'pelvis', 'waist_yaw_link',
                   rpy=('0', '0', config('waist_yaw')), condition=body_condition),
        _static_tf('g1_tf_waist_roll', 'waist_yaw_link', 'waist_roll_link',
                   xyz=('-0.0039635', '0', '0.044'),
                   rpy=(config('waist_roll'), '0', '0'), condition=body_condition),
        _static_tf('g1_tf_waist_pitch', 'waist_roll_link', 'torso_link',
                   rpy=('0', config('waist_pitch'), '0'), condition=body_condition),
        _static_tf(
            'g1_tf_d435i_mount', 'torso_link', 'd435_link',
            xyz=tuple(config('camera_' + axis) for axis in ('x', 'y', 'z')),
            rpy=tuple(config('camera_' + axis) for axis in ('roll', 'pitch', 'yaw')),
            condition=IfCondition(config('publish_camera_mount_tf')),
        ),
        # Use the URDF d435 frame as the RealSense mechanical root. The driver's
        # calibrated stream offsets and optical rotations must NOT be duplicated.
        _static_tf('g1_tf_d435i_to_camera', 'd435_link', config('camera_frame')),
    ])
