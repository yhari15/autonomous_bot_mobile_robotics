#!/usr/bin/env python3
"""
Full System Launch File
Assignment 4 Part 2 - Complete Integration

This launch file starts the complete system:
1. Gazebo simulation with Husky UR3 robot
2. EKF sensor fusion (odometry + IMU + GPS)
3. SLAM Toolbox (2D mapping)
4. Nav2 navigation stack (path planning + control)
5. RViz visualization

Use this for the complete autonomous mobile manipulation demo.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Package directories
    husky_desc_dir = FindPackageShare('husky_ur3_description')
    slam_pkg_dir = FindPackageShare('husky_ur3_slam')

    # Configuration files
    rviz_config = PathJoinSubstitution([slam_pkg_dir, 'rviz', 'full_system.rviz'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),

        # 1. Launch Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([husky_desc_dir, 'launch', 'gazebo_sim.launch.py'])
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # 2. Launch EKF sensor fusion (delayed to let Gazebo start)
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([slam_pkg_dir, 'launch', 'ekf_fusion.launch.py'])
                    ),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                ),
            ]
        ),

        # 3. Launch SLAM Toolbox (delayed to let sensors start)
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([slam_pkg_dir, 'launch', 'slam.launch.py'])
                    ),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                ),
            ]
        ),

        # 4. Launch Nav2 navigation (delayed to let SLAM initialize)
        TimerAction(
            period=12.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([slam_pkg_dir, 'launch', 'navigation.launch.py'])
                    ),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                ),
            ]
        ),

        # 5. Launch RViz (optional)
        # TimerAction(
        #     period=15.0,
        #     actions=[
        #         Node(
        #             package='rviz2',
        #             executable='rviz2',
        #             name='rviz2',
        #             output='screen',
        #             arguments=['-d', rviz_config],
        #             parameters=[{'use_sim_time': use_sim_time}],
        #             condition=launch.conditions.IfCondition(use_rviz)
        #         ),
        #     ]
        # ),
    ])
