#!/usr/bin/env python3
"""
Teleoperation Launch File for Husky UR3
Launches keyboard teleoperation for the Husky mobile base

Usage:
    ros2 launch husky_ur3_description teleop_keyboard.launch.py

Controls:
    Moving around:
       u    i    o
       j    k    l
       m    ,    .

    q/z : increase/decrease max speeds by 10%
    w/x : increase/decrease only linear speed by 10%
    e/c : increase/decrease only angular speed by 10%

    CTRL-C to quit
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Teleop Twist Keyboard Node
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        prefix='gnome-terminal --',  # Launch in separate terminal
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        teleop_keyboard_node
    ])
