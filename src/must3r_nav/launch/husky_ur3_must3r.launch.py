#!/usr/bin/env python3
"""
Launch file for Husky UR3 with ZED camera and MUSt3R 3D reconstruction
Assignment 4 Part 2

Starts:
1. Gazebo simulation with Husky UR3 robot + ZED camera
2. MUSt3R node for 3D reconstruction
3. RViz2 for visualization
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for complete system"""

    # Package directories
    husky_description_dir = FindPackageShare('husky_ur3_description')
    must3r_nav_dir = FindPackageShare('must3r_nav')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # Argument declarations
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # 1. Launch Gazebo with Husky UR3 + ZED camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    husky_description_dir,
                    'launch',
                    'gazebo_sim.launch.py'
                ])
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # 2. Launch MUSt3R bridge node (delayed to wait for Gazebo)
        # NOTE: You must also run must3r_processor.py separately in Python 3.11 env:
        #   /home/hariprasad/must3r_env/bin/python ~/ros2_ws/src/must3r_nav/scripts/must3r_processor.py
        TimerAction(
            period=5.0,  # Wait 5 seconds for Gazebo to start
            actions=[
                Node(
                    package='must3r_nav',
                    executable='must3r_bridge',
                    name='must3r_bridge_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                    }],
                    emulate_tty=True,
                )
            ]
        ),

        # 3. Launch RViz2 for visualization (delayed)
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=['-d', PathJoinSubstitution([
                        must3r_nav_dir, 'config', 'must3r_visualization.rviz'
                    ])] if os.path.exists(os.path.join(
                        os.path.dirname(__file__), '..', 'config', 'must3r_visualization.rviz'
                    )) else []
                )
            ]
        ),
    ])
