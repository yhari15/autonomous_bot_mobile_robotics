"""
MoveIt2 Launch File for Husky UR3
Assignment 4 Part 2

This launch file starts the MoveIt2 move_group node for arm planning.
Should be launched AFTER Gazebo is running.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
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
    # Package directory
    pkg_dir = get_package_share_directory('husky_ur3_description')
    
    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot description (URDF)
    urdf_file = os.path.join(pkg_dir, 'urdf', 'husky_ur3_gripper.urdf.xacro')
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # SRDF (Semantic Robot Description)
    srdf_file = os.path.join(pkg_dir, 'config', 'ur3_moveit.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Kinematics
    kinematics_yaml = load_yaml('husky_ur3_description', 'config/kinematics.yaml')
    
    # Joint limits
    joint_limits_yaml = load_yaml('husky_ur3_description', 'config/joint_limits.yaml')
    
    # MoveIt controllers configuration
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
    
    # Trajectory execution - don't manage controllers (arm_trajectory_bridge handles this)
    trajectory_execution = {
        'moveit_manage_controllers': False,
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
    
    # Sensors configuration (for collision checking with point clouds)
    sensors_yaml = {
        'sensors': [''],
    }
    
    # Move Group Node
    move_group_node = Node(
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
            sensors_yaml,
            {'use_sim_time': use_sim_time},
        ],
    )
    
    # RViz with MoveIt configuration (optional)
    rviz_config_file = os.path.join(pkg_dir, 'config', 'moveit_rviz.rviz')
    
    rviz_node = Node(
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
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        move_group_node,
        rviz_node,
    ])















