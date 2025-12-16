#!/usr/bin/env python3
"""
Complete Mars Navigation System with MoveIt Integration
Launches Gazebo + Robot + Nav2 + MoveIt2 + Arm Trajectory Bridge
For Mars world waypoint navigation
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
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
    """Generate launch description for complete Mars navigation system with MoveIt"""

    # Get package directories
    must3r_nav_dir = get_package_share_directory('must3r_nav')
    husky_ur3_desc_dir = get_package_share_directory('husky_ur3_description')
    husky_ur3_slam_dir = get_package_share_directory('husky_ur3_slam')
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

    # Check for Mars world file
    mars_world_path = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'mars.world')
    if not os.path.exists(mars_world_path):
        # Try alternate location
        mars_world_path = '/home/hariprasad/ros2_ws/mars.world'

    # Config files are in husky_ur3_slam package
    nav2_params_file = os.path.join(husky_ur3_slam_dir, 'config', 'nav2_params.yaml')
    ekf_params_file = os.path.join(husky_ur3_slam_dir, 'config', 'ekf_params.yaml')
    slam_params_file = os.path.join(husky_ur3_slam_dir, 'config', 'slam_toolbox_params.yaml')

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description (URDF)
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # SRDF (Semantic Robot Description for MoveIt)
    srdf_file = os.path.join(husky_ur3_desc_dir, 'config', 'ur3_moveit.srdf')
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
        parameters=[
            robot_description,
            {
                'use_sim_time': use_sim_time,
                'publish_frequency': 50.0,
            }
        ],
        output='screen'
    )

    # Gazebo with Mars world
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', mars_world_path, '-r', '-v', '4'],
        output='screen'
    )

    # Spawn robot at Mars start line (-3.0, 0.0)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'husky_ur3',
                '-topic', 'robot_description',
                '-x', '-3.0',
                '-y', '0.0',
                '-z', '0.2',
            ],
            output='screen'
        )]
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
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/zed_base/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed_base/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed_base/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/zed_base/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/zed_arm/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed_arm/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed_arm/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/zed_arm/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        remappings=[
            ('/odometry', '/odom'),
        ],
        output='screen'
    )

    # Robot Localization (EKF)
    ekf_node = TimerAction(
        period=7.0,
        actions=[Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        )]
    )

    # SLAM Toolbox
    slam_node = TimerAction(
        period=8.0,
        actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
        )]
    )

    # Nav2 Launch
    nav2_bringup = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file
                }.items()
            )
        ]
    )

    # MoveIt Move Group Node
    move_group_node = TimerAction(
        period=15.0,
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

    # Arm Trajectory Bridge
    arm_trajectory_bridge = TimerAction(
        period=15.0,
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

    # Publish initial pose for Nav2
    initial_pose_publisher = TimerAction(
        period=22.0,  # After Nav2 is fully ready
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

    # System ready message
    ready_message = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(
            cmd=[
                'bash', '-c',
                'echo "\\n================================================\\n'
                '✓ MARS COMPLETE SYSTEM READY\\n'
                '  - Gazebo: Mars world simulation\\n'
                '  - Nav2: Navigation stack ready\\n'
                '  - MoveIt2: Arm planning ready\\n'
                '  - SLAM: Mapping active\\n'
                '\\n'
                'To start waypoint navigation:\\n'
                '  ros2 run must3r_nav waypoint_navigator\\n'
                '================================================\\n"'
            ],
            output='screen'
        )]
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        ekf_node,
        slam_node,
        nav2_bringup,
        move_group_node,
        arm_trajectory_bridge,
        initial_pose_publisher,
        ready_message,
    ])
