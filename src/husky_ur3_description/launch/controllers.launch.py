#!/usr/bin/env python3
"""
Controllers Launch File
Launches ros2_control controller_manager and spawns controllers
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('husky_ur3_description')
    
    # Controllers config file
    controllers_config = os.path.join(pkg_dir, 'config', 'controllers.yaml')
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Controller Manager (if using ros2_control)
    # Note: This is optional - Ignition Gazebo plugins can work without it
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_config,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=None  # Always try to launch, will fail gracefully if not needed
    )
    
    # Spawn controllers using ros2 control commands
    # These will only work if ros2_control is properly configured in URDF
    
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    spawn_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Note: These are for ros2_control integration
        # Currently your setup uses Ignition Gazebo plugins directly
        # Uncomment these if you add ros2_control to your URDF
        
        # controller_manager,
        # spawn_joint_state_broadcaster,
        # spawn_diff_drive_controller,
    ])






