import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
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

    # Paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'husky_ur3_gripper.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Process URDF
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

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r', '-v', '4'],
        output='screen'
    )

    # Spawn robot
    spawn_robot = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'husky_ur3',
                       '-topic', 'robot_description',
                       '-z', '0.133'],
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
            # UR3 Joint Commands
            '/ur3_joint1_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/ur3_joint2_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/ur3_joint3_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/ur3_joint4_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/ur3_joint5_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/ur3_joint6_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
            '/rh_p12_rn_position/command@std_msgs/msg/Float64]ignition.msgs.Double',
        ],
        remappings=[
            ('/odometry', '/odom'),
        ],
        output='screen'
    )

    # Odom to TF publisher
    odom_to_tf = TimerAction(
        period=5.0,
        actions=[Node(
            package='must3r_nav',
            executable='odom_to_tf',
            name='odom_to_tf_node',
            parameters=[{'use_sim_time': use_sim_time}],
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
        odom_to_tf,
    ])
