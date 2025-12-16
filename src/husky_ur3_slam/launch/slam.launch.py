#!/usr/bin/env python3
"""
SLAM Launch File
Assignment 4 Part 2 - SLAM Implementation

This launch file starts the SLAM Toolbox node for:
- 2D mapping using laser scanner
- Loop closure detection
- Map building at 0.05m resolution

Output: /map topic with occupancy grid
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('husky_ur3_slam')

    # Configuration files
    slam_params = PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox_params.yaml'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # SLAM Toolbox Node
        # Performs online SLAM with laser scanner
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params,
                {'use_sim_time': use_sim_time}
            ],
        ),
    ])
