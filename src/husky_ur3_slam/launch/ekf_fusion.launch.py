#!/usr/bin/env python3
"""
EKF Fusion Launch File
Assignment 4 Part 2 - SLAM Implementation

This launch file starts the robot_localization EKF node to fuse:
- Odometry (wheel encoders)
- IMU (orientation and angular velocity)
- GPS (via navsat_transform)

Output: /robot_pose topic at >=10 Hz
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('husky_ur3_slam')

    # Configuration files
    ekf_params = PathJoinSubstitution([pkg_share, 'config', 'ekf_params.yaml'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Robot Localization EKF node
        # Fuses odometry + IMU data
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
            remappings=[
                ('odometry/filtered', 'robot_pose')  # Output pose topic
            ]
        ),

        # NavSat Transform node
        # Converts GPS lat/lon to local odometry frame
        # Publishes to /odometry/gps which can be fused by a second EKF
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_params],
            remappings=[
                ('gps/fix', 'gps/fix'),
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'robot_pose')  # Use EKF output
            ]
        ),
    ])
