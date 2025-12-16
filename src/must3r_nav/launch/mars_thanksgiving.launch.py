import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for Mars Thanksgiving World"""

    # Get package directories
    must3r_nav_dir = get_package_share_directory('must3r_nav')
    husky_ur3_desc_dir = get_package_share_directory('husky_ur3_description')

    # Get parent directory (share/) for Gazebo resource path
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

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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
        cmd=['ign', 'gazebo', world_file, '-r', '-v', '4'],
        output='screen'
    )

    # Spawn robot at Mars start line (-2.0, 0.0)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'husky_ur3',
                '-topic', 'robot_description',
                '-x', '-2.0',
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

    # Odom to TF publisher
    odom_to_tf = TimerAction(
        period=7.0,
        actions=[Node(
            package='must3r_nav',
            executable='odom_to_tf',
            name='odom_to_tf_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )]
    )

    # Info message
    info_message = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=[
                'bash', '-c',
                'echo "\\n================================================\\n'
                'Mars Thanksgiving World Launched!\\n'
                'Robot spawned at start line (-2.0, 0.0)\\n'
                'Table center: (3.0, 0.0)\\n'
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
        odom_to_tf,
        info_message,
    ])
