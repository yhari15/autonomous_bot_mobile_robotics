#!/usr/bin/env python3
"""
Mars Navigation with Nav2 Launch File
Launches Mars Thanksgiving world with full Nav2 navigation stack
for autonomous waypoint following around the table
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml_file(file_path):
    """Safely load a YAML file."""
    try:
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Warning: Could not load {file_path}: {e}")
        return {}


def generate_launch_description():
    """Generate launch description for Mars Navigation with Nav2"""

    # Get package directories
    must3r_nav_dir = get_package_share_directory('must3r_nav')
    husky_ur3_desc_dir = get_package_share_directory('husky_ur3_description')
    share_dir = os.path.dirname(husky_ur3_desc_dir)

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
    urdf_file = os.path.join(husky_ur3_desc_dir, 'urdf', 'husky_ur3_gripper.urdf.xacro')
    world_file = os.path.join(must3r_nav_dir, 'worlds', 'mars_thanksgiving.world')
    nav2_params_file = os.path.join(must3r_nav_dir, 'config', 'mars_nav2_params.yaml')

    # Behavior tree XML files
    nav2_bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')
    default_bt_xml_path = os.path.join(
        nav2_bt_navigator_dir,
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml'
    )
    default_bt_xml_through_poses_path = os.path.join(
        nav2_bt_navigator_dir,
        'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery.xml'
    )

    # MoveIt configuration files
    srdf_file = os.path.join(husky_ur3_desc_dir, 'config', 'ur3_moveit.srdf')
    kinematics_file = os.path.join(husky_ur3_desc_dir, 'config', 'kinematics.yaml')
    joint_limits_file = os.path.join(husky_ur3_desc_dir, 'config', 'joint_limits.yaml')
    moveit_controllers_file = os.path.join(husky_ur3_desc_dir, 'config', 'moveit_controllers.yaml')

    # Load MoveIt configurations
    srdf_content = ""
    try:
        with open(srdf_file, 'r') as f:
            srdf_content = f.read()
    except Exception as e:
        print(f"Warning: Could not load SRDF: {e}")
    
    robot_description_semantic = {'robot_description_semantic': srdf_content} if srdf_content else {}
    kinematics_yaml = load_yaml_file(kinematics_file)
    joint_limits_yaml = load_yaml_file(joint_limits_file)
    moveit_controllers = load_yaml_file(moveit_controllers_file)

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    # Process URDF with xacro
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file], on_stderr='ignore'),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
        }],
        output='screen'
    )

    # Gazebo Simulation
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r', '-v', '4'],
        output='screen'
    )

    # Spawn robot
    spawn_robot = TimerAction(
        period=8.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'husky_ur3',
                '-topic', 'robot_description',
                '-x', '-3.0',
                '-y', '0.0',
                '-z', '0.15',
            ],
            output='screen'
        )]
    )

    # ROS-Gazebo Bridge (including arm joints)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Core topics
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # Laser scan
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            # ZED Base Camera
            '/zed_base/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed_base/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed_base/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/zed_base/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # ZED Arm Camera
            '/zed_arm/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed_arm/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed_arm/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/zed_arm/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # UR3 Arm Joint Position Commands (ROS -> Ignition)
            '/ur3_joint1_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/ur3_joint2_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/ur3_joint3_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/ur3_joint4_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/ur3_joint5_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/ur3_joint6_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            # Gripper Command
            '/rh_p12_rn_position/command@std_msgs/msg/Float64]ignition.msgs.Double',
        ],
        remappings=[
            ('/odometry', '/odom'),
        ],
        output='screen'
    )

    # Odom to TF publisher (publishes odom->base_link)
    odom_to_tf = TimerAction(
        period=10.0,
        actions=[Node(
            package='must3r_nav',
            executable='odom_to_tf',
            name='odom_to_tf_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )]
    )

    # Static transform: map -> odom
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Arm Trajectory Bridge
    arm_trajectory_bridge = TimerAction(
        period=10.0,
        actions=[Node(
            package='must3r_nav',
            executable='arm_trajectory_bridge',
            name='arm_trajectory_bridge',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )]
    )

    # MoveIt planning configuration
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

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.05,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # MoveIt move_group node
    move_group_node = TimerAction(
        period=18.0,
        actions=[Node(
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
        )]
    )

    # Nav2 Stack
    nav2_nodes_timer = TimerAction(
        period=12.0,
        actions=[
            # Nav2 Controller Server
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params_file],
                remappings=[('/cmd_vel', '/cmd_vel_nav')]
            ),
            # Nav2 Planner Server
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params_file]
            ),
            # Nav2 Behavior Server
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params_file]
            ),
            # Nav2 BT Navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[
                    nav2_params_file,
                    {
                        'default_nav_to_pose_bt_xml': default_bt_xml_path,
                        'default_nav_through_poses_bt_xml': default_bt_xml_through_poses_path
                    }
                ]
            ),
            # Nav2 Waypoint Follower
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[nav2_params_file]
            ),
            # Nav2 Velocity Smoother
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[nav2_params_file],
                remappings=[('/cmd_vel', '/cmd_vel_nav'),
                           ('/cmd_vel_smoothed', '/cmd_vel')]
            ),
            # Nav2 Lifecycle Manager
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': [
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother'
                    ]}
                ]
            ),
        ]
    )

    # Info message
    info_message = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=[
                'bash', '-c',
                'echo "\n'
                '================================================\n'
                'Mars Navigation System with Nav2\n'
                '================================================\n'
                'Gazebo starting...\n'
                'Robot will spawn in 8 seconds at (-3.0, 0.0)\n'
                'Nav2 stack will start in 12 seconds\n'
                'Table center: (3.0, 0.0)\n'
                '\n'
                'Camera Topics:\n'
                '  - /zed_base/left/image_raw  (Husky base camera)\n'
                '  - /zed_arm/left/image_raw   (UR3 arm camera)\n'
                '\n'
                'Navigation Topics:\n'
                '  - /scan          (Laser scanner for obstacles)\n'
                '  - /odom          (Odometry)\n'
                '  - /cmd_vel       (Velocity commands)\n'
                '\n'
                'WAIT for robot to spawn before running:\n'
                '  ros2 run must3r_nav waypoint_navigator\n'
                '================================================\n"'
            ],
            output='screen'
        )]
    )

    ready_message = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(
            cmd=[
                'bash', '-c',
                'echo "\n'
                '================================================\n'
                'SYSTEM READY - You can now run:\n'
                '  ros2 run must3r_nav waypoint_navigator\n'
                '================================================\n"'
            ],
            output='screen'
        )]
    )

    # Publish initial pose
    initial_pose_publisher = TimerAction(
        period=22.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--once', '/initialpose',
                'geometry_msgs/msg/PoseWithCovarianceStamped',
                '{header: {frame_id: "map"}, '
                'pose: {pose: {position: {x: -3.0, y: 0.0, z: 0.0}, '
                'orientation: {w: 1.0}}}}'
            ],
            output='screen'
        )]
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        odom_to_tf,
        static_tf_map_to_odom,
        arm_trajectory_bridge,
        move_group_node,
        nav2_nodes_timer,
        info_message,
        ready_message,
        initial_pose_publisher,
    ])
