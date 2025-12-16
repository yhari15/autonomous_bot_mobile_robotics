#!/usr/bin/env python3
"""
Waypoint Navigator for Mars Thanksgiving Table Mapping
Navigates Husky through 30 waypoints using Nav2 and captures images for MUST3R
Uses MoveIt2 for arm motion planning

Usage:
    ros2 run must3r_nav waypoint_navigator

Requirements:
    - mars_nav2.launch.py must be running
    - Nav2 stack must be active
    - MoveIt2 move_group must be running
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import yaml
import math
import time
import cv2
from pathlib import Path


class WaypointNavigator(Node):
    """Navigate through waypoints for MuST3R 3D reconstruction."""

    def __init__(self):
        super().__init__('waypoint_navigator')

        # Parameters
        self.declare_parameter('waypoints_file',
            str(Path.home() / 'ros2_ws/src/must3r_nav/config/navigation_waypoints.yaml'))
        self.declare_parameter('output_dir',
            str(Path.home() / 'ros2_ws/must3r_captures'))
        self.declare_parameter('capture_delay', 3.0)  # seconds to wait at each waypoint

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.capture_delay = self.get_parameter('capture_delay').value

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Load waypoints
        self.waypoints = self.load_waypoints()
        self.arm_configs = self.load_arm_configs()
        self.current_waypoint = 0

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Captured images for MUST3R
        self.captured_images = []
        self.base_camera_image = None  # ZED base camera
        self.arm_camera_image = None   # ZED arm camera

        # Action client for Nav2 navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # MoveIt/Arm control - uses FollowJointTrajectory action
        # This works with both arm_trajectory_bridge and MoveIt's controller manager
        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                      '/arm_controller/follow_joint_trajectory')

        # Track joint states for monitoring
        self.current_joint_states = {}
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Subscribers for cameras (ZED 2 stereo - using left images)
        self.base_camera_sub = self.create_subscription(
            Image, '/zed_base/left/image_raw', self.base_camera_callback, 10)
        self.arm_camera_sub = self.create_subscription(
            Image, '/zed_arm/left/image_raw', self.arm_camera_callback, 10)

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info('Waiting for Nav2 action server...')

        # Wait for Nav2 server
        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 action server not available!')
            self.get_logger().error('Make sure mars_nav2.launch.py is running')
        else:
            self.get_logger().info('Nav2 action server is ready!')

    def load_waypoints(self):
        """Load waypoints from YAML file."""
        try:
            with open(self.waypoints_file, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []

    def load_arm_configs(self):
        """Load arm configurations from YAML file."""
        try:
            with open(self.waypoints_file, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('arm_configs', {})
        except Exception as e:
            self.get_logger().error(f'Failed to load arm configs: {e}')
            return {}

    def base_camera_callback(self, msg):
        """Store latest base camera image."""
        self.base_camera_image = msg

    def arm_camera_callback(self, msg):
        """Store latest arm camera image."""
        self.arm_camera_image = msg

    def joint_state_callback(self, msg):
        """Track current joint states."""
        for name, position in zip(msg.name, msg.position):
            self.current_joint_states[name] = position

    def create_goal_pose(self, waypoint):
        """Create a PoseStamped from waypoint data."""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        pose = waypoint['pose']
        goal.pose.position.x = pose['x']
        goal.pose.position.y = pose['y']
        goal.pose.position.z = pose.get('z', 0.0)

        # Convert yaw to quaternion
        yaw = pose['yaw']
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)

        return goal

    def set_arm_config(self, config_name):
        """Move arm to specified configuration using action client."""
        if config_name not in self.arm_configs:
            self.get_logger().warn(f'Unknown arm config: {config_name}')
            return False

        config = self.arm_configs[config_name]
        joint_positions = config['joint_positions']

        # Create trajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'ur3_joint1',
            'ur3_joint2',
            'ur3_joint3',
            'ur3_joint4',
            'ur3_joint5',
            'ur3_joint6'
        ]

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(seconds=4).to_msg()  # 4 seconds to move

        goal.trajectory.points.append(point)

        self.get_logger().info(f'Moving arm to config: {config_name}')

        # Send goal to arm controller (availability will be checked by goal acceptance)
        send_goal_future = self.arm_client.send_goal_async(goal)

        # Wait for send_goal to complete
        start_time = time.time()
        while not send_goal_future.done():
            if time.time() - start_time > 5.0:
                self.get_logger().error(f'Timed out sending arm goal for {config_name}')
                return False
            time.sleep(0.1)

        try:
            goal_handle = send_goal_future.result()
        except Exception as e:
            self.get_logger().error(f'Error sending arm goal: {e}')
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f'Arm goal rejected for config: {config_name}')
            return False

        self.get_logger().info(f'Arm goal accepted, moving to: {config_name}')

        # Wait for result
        result_future = goal_handle.get_result_async()
        start_time = time.time()

        while not result_future.done():
            if time.time() - start_time > 15.0:  # 15 seconds for arm movement
                self.get_logger().warn(f'Arm movement to {config_name} timed out')
                return False
            time.sleep(0.2)

        try:
            result = result_future.result()
            if result.status == 4:  # SUCCEEDED
                self.get_logger().info(f'Arm successfully moved to {config_name}')
                return True
            else:
                self.get_logger().warn(f'Arm movement ended with status: {result.status}')
                return False
        except Exception as e:
            self.get_logger().error(f'Arm movement error: {e}')
            return False

    def capture_images(self, waypoint_id):
        """Capture images from both ZED cameras."""
        captures = []

        # Wait for fresh images
        time.sleep(0.5)

        # Capture base camera (Husky)
        if self.base_camera_image is not None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(
                    self.base_camera_image, 'bgr8')
                filename = self.output_dir / f'wp{waypoint_id:02d}_base.jpg'
                cv2.imwrite(str(filename), cv_image)
                captures.append(str(filename))
                self.get_logger().info(f'Captured base: {filename}')
            except Exception as e:
                self.get_logger().error(f'Base camera capture failed: {e}')

        # Capture arm camera
        if self.arm_camera_image is not None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(
                    self.arm_camera_image, 'bgr8')
                filename = self.output_dir / f'wp{waypoint_id:02d}_arm.jpg'
                cv2.imwrite(str(filename), cv_image)
                captures.append(str(filename))
                self.get_logger().info(f'Captured arm: {filename}')
            except Exception as e:
                self.get_logger().error(f'Arm camera capture failed: {e}')

        self.captured_images.extend(captures)
        return captures

    def navigate_to_waypoint(self, waypoint):
        """Navigate to a single waypoint using Nav2."""
        wp_id = waypoint['id']
        self.get_logger().info(
            f"Navigating to waypoint {wp_id}: {waypoint['description']}")

        self.get_logger().info(f'DEBUG: Creating goal for waypoint {wp_id}...')
        goal = NavigateToPose.Goal()

        self.get_logger().info(f'DEBUG: Creating goal pose for waypoint {wp_id}...')
        goal.pose = self.create_goal_pose(waypoint)

        self.get_logger().info(f'DEBUG: Goal pose created, about to send goal...')
        # Action server availability already checked in __init__
        # Send goal - use executor's spin_until_future_complete for reliability
        self.get_logger().info(f'Sending goal for waypoint {wp_id}...')
        send_goal_future = self.nav_client.send_goal_async(goal)

        # Spin until send_goal completes (with timeout)
        self.get_logger().info(f'Waiting for goal acceptance...')
        start_time = time.time()
        while not send_goal_future.done():
            if time.time() - start_time > 10.0:
                self.get_logger().error(f'Timed out waiting for goal acceptance after 10s')
                return False
            if time.time() - start_time > 2.0 and int(time.time() - start_time) % 2 == 0:
                self.get_logger().info(f'Still waiting for goal acceptance... ({int(time.time() - start_time)}s)')
            time.sleep(0.1)

        self.get_logger().info(f'Got response from send_goal_async, checking result...')
        try:
            goal_handle = send_goal_future.result()
        except Exception as e:
            self.get_logger().error(f'Error getting goal handle: {e}')
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected for waypoint {wp_id}')
            return False

        self.get_logger().info(f'Goal accepted, navigating to waypoint {wp_id}')

        # Wait for result
        result_future = goal_handle.get_result_async()

        # Spin until result completes (with timeout)
        start_time = time.time()
        last_log_time = 0

        while not result_future.done():
            elapsed = time.time() - start_time

            if elapsed > 90.0:
                self.get_logger().error(f'Navigation to waypoint {wp_id} timed out after 90s')
                try:
                    goal_handle.cancel_goal_async()
                except:
                    pass
                return False

            # Log progress every 5 seconds
            if int(elapsed) - last_log_time >= 5:
                self.get_logger().info(f'Still navigating to waypoint {wp_id}... ({int(elapsed)}s elapsed)')
                last_log_time = int(elapsed)

            time.sleep(0.2)

        try:
            result = result_future.result()
            if result.status == 4:  # SUCCEEDED
                self.get_logger().info(f'Successfully reached waypoint {wp_id}!')
                return True
            else:
                self.get_logger().warn(f'Navigation ended with status: {result.status}')
                return False
        except Exception as e:
            self.get_logger().error(f'Navigation error for waypoint {wp_id}: {e}')
            return False

    def run_mapping_sequence(self):
        """Execute the full mapping sequence through all waypoints."""
        self.get_logger().info('='*50)
        self.get_logger().info('STARTING MARS MAPPING SEQUENCE')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
        self.get_logger().info('='*50)

        for i, waypoint in enumerate(self.waypoints):
            wp_id = waypoint['id']
            ring = waypoint.get('ring', 1)
            arm_config = waypoint.get('arm_config', 'medium')

            self.get_logger().info(f'\n--- Waypoint {wp_id}/{len(self.waypoints)} ---')
            self.get_logger().info(f'Ring: {ring}, Config: {arm_config}')

            # Navigate to waypoint using Nav2
            success = self.navigate_to_waypoint(waypoint)
            if not success:
                self.get_logger().warn(f'Navigation to waypoint {wp_id} failed, continuing...')
                # Don't skip - still try to capture from current position
            else:
                self.get_logger().info(f'Reached waypoint {wp_id}!')

            # Set arm configuration
            arm_success = self.set_arm_config(arm_config)
            if arm_success:
                self.get_logger().info(f'Arm moved to {arm_config} config')
                time.sleep(1.0)  # Let arm settle
            else:
                self.get_logger().warn(f'Arm movement failed, continuing with current pose')

            # Wait at waypoint for stabilization
            self.get_logger().info(f'Waiting {self.capture_delay}s at waypoint...')
            time.sleep(self.capture_delay)

            # Capture images from both cameras
            captures = self.capture_images(wp_id)
            self.get_logger().info(f'Captured {len(captures)} images')

        self.get_logger().info('='*50)
        self.get_logger().info('MARS MAPPING SEQUENCE COMPLETE!')
        self.get_logger().info(f'Total images captured: {len(self.captured_images)}')
        self.get_logger().info(f'Images saved to: {self.output_dir}')
        self.get_logger().info('='*50)

        # Save image list for MUST3R
        image_list_file = self.output_dir / 'image_list.txt'
        with open(image_list_file, 'w') as f:
            for img_path in self.captured_images:
                f.write(f'{img_path}\n')
        self.get_logger().info(f'Image list saved to: {image_list_file}')


async def main_async(args=None):
    """Async main function for running waypoint navigation."""
    import asyncio
    import threading
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)

    navigator = WaypointNavigator()

    # Use MultiThreadedExecutor for better action client handling
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(navigator)

    # Spin the executor in a separate thread so callbacks can be processed
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Run the mapping sequence
        navigator.run_mapping_sequence()
    except KeyboardInterrupt:
        navigator.get_logger().info('Interrupted by user')
    finally:
        executor.shutdown()
        navigator.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


def main(args=None):
    """Entry point for waypoint navigator."""
    import asyncio
    asyncio.run(main_async(args))


if __name__ == '__main__':
    main()
