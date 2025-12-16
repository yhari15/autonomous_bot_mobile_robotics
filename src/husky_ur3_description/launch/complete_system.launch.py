#!/usr/bin/env python3
"""
Complete System Launch File
Launches Gazebo + Robot + MoveIt2 + Arm Trajectory Bridge
For Husky UR3 with gripper
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    """Load a YAML file from a package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading {absolute_file_path}: {e}")
        return {}


def generate_launch_description():
    """Generate launch description for complete system"""

    # Get package directories
    pkg_dir = get_package_share_directory('husky_ur3_description')
    share_dir = os.path.dirname(pkg_dir)

    # Set Gazebo resource paths
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=share_dir
    )

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=share_dir
    )

    # File paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'husky_ur3_gripper.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.sdf')

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description (URDF)
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # SRDF (Semantic Robot Description for MoveIt)
    srdf_file = os.path.join(pkg_dir, 'config', 'ur3_moveit.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # MoveIt configuration
    kinematics_yaml = load_yaml('husky_ur3_description', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('husky_ur3_description', 'config/joint_limits.yaml')
    moveit_controllers = load_yaml('husky_ur3_description', 'config/moveit_controllers.yaml')

    # Planning configuration (OMPL)
    ompl_planning_yaml = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                               'default_planner_request_adapters/ResolveConstraintFrames '
                               'default_planner_request_adapters/FixWorkspaceBounds '
                               'default_planner_request_adapters/FixStartStateBounds '
                               'default_planner_request_adapters/FixStartStateCollision '
                               'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }

    # Trajectory execution
    trajectory_execution = {
        'moveit_manage_controllers': False,  # arm_trajectory_bridge handles this
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.05,
    }

    # Planning scene monitor
    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

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

    # Spawn robot in Gazebo (delayed to let Gazebo start)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', 'husky_ur3', '-topic', 'robot_description', '-z', '0.133'],
                output='screen'
            )
        ]
    )

    # ROS-Gazebo Bridge for base topics
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

    # MoveIt Move Group Node (delayed to let robot spawn)
    move_group_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                name='move_group',
                output='screen',
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    ompl_planning_yaml,
                    trajectory_execution,
                    planning_scene_monitor,
                    moveit_controllers,
                    joint_limits_yaml,
                    {'use_sim_time': use_sim_time},
                ],
            )
        ]
    )

    # Arm Trajectory Bridge (bridges MoveIt to Gazebo)
    arm_trajectory_bridge = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='must3r_nav',
                executable='arm_trajectory_bridge',
                name='arm_trajectory_bridge',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )

    # RViz with MoveIt configuration (optional)
    rviz_config_file = os.path.join(pkg_dir, 'config', 'moveit_rviz.rviz')

    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_moveit',
                output='screen',
                arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    {'use_sim_time': use_sim_time},
                ],
            )
        ]
    )

    # Info message
    info_message = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=[
                'bash', '-c',
                'echo "\\n========================================\\n'
                'Complete System Launching\\n'
                '  - Gazebo Simulation\\n'
                '  - MoveIt2 Motion Planning\\n'
                '  - Arm Trajectory Bridge\\n'
                '  - RViz2 Visualization\\n'
                '========================================\\n"'
            ],
            output='screen'
        )]
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo_server,
        gazebo_gui,
        spawn_robot,
        bridge,
        move_group_node,
        arm_trajectory_bridge,
        rviz_node,
        info_message,
    ])
