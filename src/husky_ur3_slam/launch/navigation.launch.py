#!/usr/bin/env python3
"""
Navigation Launch File
Assignment 4 Part 2 - Path Planning & Control

This launch file starts the Nav2 navigation stack for:
- Global path planning (A* algorithm)
- Local path planning (DWA - Dynamic Window Approach)
- Obstacle avoidance using laser scanner
- Autonomous navigation to goal poses

Requirements: SLAM must be running (map available)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Package directories
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    pkg_share = FindPackageShare('husky_ur3_slam')

    # Configuration files
    nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='False')
    use_respawn = LaunchConfiguration('use_respawn', default='False')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),

        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Use composed bringup if True'
        ),

        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Whether to respawn if a node crashes'
        ),

        # Include Nav2 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': nav2_params,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
            }.items()
        ),
    ])
