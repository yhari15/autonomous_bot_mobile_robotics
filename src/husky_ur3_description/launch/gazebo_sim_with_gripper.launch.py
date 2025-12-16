#!/usr/bin/env python3
"""
Launch file for Husky+UR3+Gripper in Ignition Gazebo with warehouse world
This version uses pre-processed URDF to ensure gripper is included
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('husky_ur3_description')
    
    # Get parent directory (share/) so model://husky_ur3_description/... resolves correctly
    share_dir = os.path.dirname(pkg_dir)
    
    # Set Gazebo resource paths to find meshes
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=share_dir
    )
    
    # Also set IGN_GAZEBO_RESOURCE_PATH for backwards compatibility
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=share_dir
    )
    
    # Paths - use pre-processed URDF to ensure gripper is included
    urdf_file = os.path.join(pkg_dir, 'urdf', 'husky_ur3_gripper_processed.urdf')
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.sdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Read pre-processed URDF file
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {
            'use_sim_time': use_sim_time,
            'ignore_timestamp': False
        }],
        output='screen'
    )
    
    # Gazebo Server
    gazebo_server = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r', '-v', '4'],
        output='screen'
    )
    
    # Gazebo GUI
    gazebo_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', 'husky_ur3', '-topic', 'robot_description', '-z', '0.133'],
                output='screen'
            )
        ]
    )
    
    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/rh_p12_rn_position/command@std_msgs/msg/Float64]ignition.msgs.Double',
        ],
        remappings=[
            ('/odometry', '/odom'),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo_server,
        gazebo_gui,
        spawn_robot,
        bridge
    ])


