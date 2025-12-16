#!/usr/bin/env python3
"""
Autonomous Demo Launch File
Assignment 4 Part 2 - Complete Integration Demo

This launch file starts the complete autonomous mobile manipulation demo:
1. Full system (Gazebo + EKF + SLAM + Nav2)
2. Object detection
3. Manipulation controller
4. Mission controller

This is the complete demo for assignment submission.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Package directories
    slam_pkg_dir = FindPackageShare('husky_ur3_slam')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    exploration_time = LaunchConfiguration('exploration_time', default='180.0')
    objects_to_manipulate = LaunchConfiguration('objects_to_manipulate', default='2')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'exploration_time',
            default_value='180.0',
            description='Time to spend exploring (seconds)'
        ),

        DeclareLaunchArgument(
            'objects_to_manipulate',
            default_value='2',
            description='Number of objects to manipulate during mission'
        ),

        # 1. Launch full system (Gazebo + EKF + SLAM + Nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([slam_pkg_dir, 'launch', 'full_system.launch.py'])
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # 2. Object Detector (delayed to let system initialize)
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='husky_ur3_slam',
                    executable='object_detector.py',
                    name='object_detector',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                ),
            ]
        ),

        # 3. Simple Manipulator (delayed to let system initialize)
        TimerAction(
            period=16.0,
            actions=[
                Node(
                    package='husky_ur3_slam',
                    executable='simple_manipulator.py',
                    name='simple_manipulator',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'manipulation_enabled': True,
                        'max_objects_to_manipulate': objects_to_manipulate
                    }]
                ),
            ]
        ),

        # 4. Mission Controller (delayed to let everything initialize)
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='husky_ur3_slam',
                    executable='mission_controller.py',
                    name='mission_controller',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'exploration_time': exploration_time,
                        'objects_to_manipulate': objects_to_manipulate,
                        'return_to_start': True
                    }]
                ),
            ]
        ),
    ])
