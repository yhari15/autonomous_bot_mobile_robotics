#!/usr/bin/env python3
"""
Waypoint Navigator for Mars Thanksgiving Table Mapping
Navigates Husky from start to end while capturing images for MUSt3R
Saves PCD file at the end.

Usage:
    ros2 run must3r_nav waypoint_navigator

Requirements:
    - mars_nav2.launch.py must be running
    - must3r_bridge node must be running
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
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
        self.declare_parameter('capture_delay', 2.0)  # seconds to wait at each waypoint

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
        self.base_camera_image = None
        self.arm_camera_image = None

        # Action client for Nav2 navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Service client to save PCD file
        self.save_pcd_client = self.create_client(Trigger, '/must3r/save_pcd')

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

        goal = NavigateToPose.Goal()
        goal.pose = self.create_goal_pose(waypoint)

        self.get_logger().info(f'Sending goal for waypoint {wp_id}...')
        send_goal_future = self.nav_client.send_goal_async(goal)

        # Spin until send_goal completes (with timeout)
        start_time = time.time()
        while not send_goal_future.done():
            if time.time() - start_time > 10.0:
                self.get_logger().error(f'Timed out waiting for goal acceptance')
                return False
            time.sleep(0.1)

        try:
            goal_handle = send_goal_future.result()
        except Exception as e:
            self.get_logger().error(f'Error sending goal: {e}')
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected for waypoint {wp_id}')
            return False

        self.get_logger().info(f'Goal accepted, navigating to waypoint {wp_id}')

        # Wait for result
        result_future = goal_handle.get_result_async()
        start_time = time.time()

        while not result_future.done():
            elapsed = time.time() - start_time
            if elapsed > 120.0:  # 2 minute timeout
                self.get_logger().error(f'Navigation timed out after 120s')
                try:
                    goal_handle.cancel_goal_async()
                except:
                    pass
                return False

            # Log progress every 10 seconds
            if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                self.get_logger().info(f'Navigating... ({int(elapsed)}s elapsed)')
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
            self.get_logger().error(f'Navigation error: {e}')
            return False

    def save_pcd_file(self):
        """Call MUSt3R bridge to save accumulated point cloud as PCD."""
        self.get_logger().info('='*50)
        self.get_logger().info('SAVING POINT CLOUD DATA')
        self.get_logger().info('='*50)

        if not self.save_pcd_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('MUSt3R save_pcd service not available!')
            self.get_logger().error('Make sure must3r_bridge is running')
            return False

        request = Trigger.Request()
        future = self.save_pcd_client.call_async(request)

        # Wait for service response
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 30.0:
                self.get_logger().error('Timed out waiting for PCD save')
                return False
            time.sleep(0.1)

        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'PCD saved: {response.message}')
                return True
            else:
                self.get_logger().error(f'PCD save failed: {response.message}')
                return False
        except Exception as e:
            self.get_logger().error(f'PCD save error: {e}')
            return False

    def run_mapping_sequence(self):
        """Execute the full mapping sequence through all waypoints."""
        self.get_logger().info('='*50)
        self.get_logger().info('STARTING MARS TABLE MAPPING')
        self.get_logger().info(f'Path: Start → Table → End')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
        self.get_logger().info('='*50)

        successful_waypoints = 0

        for i, waypoint in enumerate(self.waypoints):
            wp_id = waypoint['id']

            self.get_logger().info(f'\n--- Waypoint {wp_id}/{len(self.waypoints)} ---')
            self.get_logger().info(f'Description: {waypoint.get("description", "N/A")}')

            # Navigate to waypoint using Nav2
            success = self.navigate_to_waypoint(waypoint)
            if success:
                successful_waypoints += 1
                self.get_logger().info(f'Reached waypoint {wp_id}!')
            else:
                self.get_logger().warn(f'Navigation to waypoint {wp_id} failed, continuing...')

            # Wait at waypoint for image capture
            self.get_logger().info(f'Capturing at waypoint ({self.capture_delay}s)...')
            time.sleep(self.capture_delay)

            # Capture images
            captures = self.capture_images(wp_id)
            self.get_logger().info(f'Captured {len(captures)} images')

        self.get_logger().info('='*50)
        self.get_logger().info('NAVIGATION COMPLETE!')
        self.get_logger().info(f'Successful waypoints: {successful_waypoints}/{len(self.waypoints)}')
        self.get_logger().info(f'Total images captured: {len(self.captured_images)}')
        self.get_logger().info('='*50)

        # Give MUSt3R time to process final images
        self.get_logger().info('Waiting for MUSt3R to finish processing (10s)...')
        time.sleep(10.0)

        # Save PCD file
        pcd_success = self.save_pcd_file()

        if pcd_success:
            self.get_logger().info('='*50)
            self.get_logger().info('SUCCESS! Point cloud saved as PCD file')
            self.get_logger().info('Check ~/ros2_ws/pointclouds/ for the output')
            self.get_logger().info('='*50)
        else:
            self.get_logger().warn('PCD save may have failed - check must3r_bridge logs')

        # Save image list for reference
        image_list_file = self.output_dir / 'image_list.txt'
        with open(image_list_file, 'w') as f:
            for img_path in self.captured_images:
                f.write(f'{img_path}\n')
        self.get_logger().info(f'Image list saved to: {image_list_file}')


def main(args=None):
    """Entry point for waypoint navigator."""
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


if __name__ == '__main__':
    main()
