#!/usr/bin/env python3
"""
Vision-based Navigator Node
Uses obstacle detection from MUSt3R 3D reconstruction for navigation

Assignment 4 Part 2 - Vision-based Navigation (15 points)
- Obstacle detection from reconstructed point clouds ✓
- Path planning - robot navigates around obstacles
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
import struct
import math

from .point_cloud_processor import PointCloudProcessor


class VisionNavigatorNode(Node):
    """
    Vision-based navigation using MUSt3R 3D reconstruction
    
    Implements reactive obstacle avoidance:
    - Processes obstacle point cloud
    - Computes safe directions
    - Publishes velocity commands to avoid obstacles
    """
    
    def __init__(self):
        super().__init__('vision_navigator')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('obstacle_distance_threshold', 1.5)
        self.declare_parameter('critical_distance', 0.5)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('enabled', True)
        
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.obs_threshold = self.get_parameter('obstacle_distance_threshold').value
        self.critical_dist = self.get_parameter('critical_distance').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.enabled = self.get_parameter('enabled').value
        
        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.obstacles = []
        self.last_cmd_time = self.get_clock().now()
        
        # Point cloud processor
        self.processor = PointCloudProcessor(
            voxel_size=0.1,
            ground_threshold=0.2,
            min_obstacle_height=0.15,
            max_obstacle_height=1.5
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/must3r/point_cloud',
            self.pointcloud_callback,
            10
        )
        
        # Can also subscribe to pre-processed obstacles
        self.obstacles_sub = self.create_subscription(
            PointCloud2,
            '/obstacles/points',
            self.obstacles_callback,
            10
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Navigation timer
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)  # 10 Hz
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Vision Navigator Node Started")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Max speeds: linear={self.max_linear}, angular={self.max_angular}")
        self.get_logger().info(f"Obstacle threshold: {self.obs_threshold}m")
        self.get_logger().info(f"Critical distance: {self.critical_dist}m")
        self.get_logger().info(f"Goal: ({self.goal_x}, {self.goal_y})")
        self.get_logger().info(f"Navigation enabled: {self.enabled}")
        self.get_logger().info("")
        self.get_logger().info("Commands:")
        self.get_logger().info("  ros2 param set /vision_navigator enabled true  (start)")
        self.get_logger().info("  ros2 param set /vision_navigator enabled false (stop)")
        self.get_logger().info("  ros2 param set /vision_navigator goal_x 10.0")
        self.get_logger().info("=" * 60)
    
    def odom_callback(self, msg: Odometry):
        """Update robot position from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Process MUSt3R point cloud for obstacles"""
        try:
            points = self.pointcloud2_to_array(msg)
            if len(points) > 0:
                result = self.processor.process(points)
                self.obstacles = self.processor.detect_obstacle_clusters(
                    result['obstacles'],
                    cluster_tolerance=0.2,
                    min_cluster_size=5
                )
                # Compute distances
                robot_pos = np.array([self.robot_x, self.robot_y, 0])
                self.obstacles = self.processor.compute_obstacle_distances(
                    self.obstacles, robot_pos
                )
        except Exception as e:
            self.get_logger().error(f"Point cloud processing error: {e}")
    
    def obstacles_callback(self, msg: PointCloud2):
        """Process pre-processed obstacle point cloud"""
        try:
            points = self.pointcloud2_to_array(msg)
            if len(points) > 0:
                self.obstacles = self.processor.detect_obstacle_clusters(
                    points,
                    cluster_tolerance=0.2,
                    min_cluster_size=5
                )
                robot_pos = np.array([self.robot_x, self.robot_y, 0])
                self.obstacles = self.processor.compute_obstacle_distances(
                    self.obstacles, robot_pos
                )
        except Exception as e:
            pass  # Silently handle - may not have obstacles
    
    def pointcloud2_to_array(self, msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 to numpy array"""
        point_step = msg.point_step
        num_points = msg.width * msg.height
        
        if num_points == 0:
            return np.array([]).reshape(0, 3)
        
        points = np.zeros((num_points, 3), dtype=np.float32)
        
        for i in range(num_points):
            offset = i * point_step
            x = struct.unpack_from('f', msg.data, offset)[0]
            y = struct.unpack_from('f', msg.data, offset + 4)[0]
            z = struct.unpack_from('f', msg.data, offset + 8)[0]
            points[i] = [x, y, z]
        
        valid_mask = np.isfinite(points).all(axis=1)
        return points[valid_mask]
    
    def navigation_loop(self):
        """Main navigation control loop"""
        # Check if enabled
        try:
            self.enabled = self.get_parameter('enabled').value
            self.goal_x = self.get_parameter('goal_x').value
            self.goal_y = self.get_parameter('goal_y').value
        except:
            pass
        
        if not self.enabled:
            return
        
        # Compute goal direction
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        goal_distance = math.sqrt(dx*dx + dy*dy)
        goal_angle = math.atan2(dy, dx)
        
        # Check if goal reached
        if goal_distance < self.goal_tolerance:
            self.get_logger().info("Goal reached!")
            self.publish_stop()
            return
        
        # Compute angle difference to goal
        angle_diff = self.normalize_angle(goal_angle - self.robot_yaw)
        
        # Check for obstacles
        closest_obstacle = None
        closest_distance = float('inf')
        obstacle_angle = 0.0
        
        for obs in self.obstacles:
            if obs['distance'] < closest_distance:
                closest_distance = obs['distance']
                closest_obstacle = obs
                # Compute angle to obstacle
                obs_dx = obs['centroid'][0] - self.robot_x
                obs_dy = obs['centroid'][1] - self.robot_y
                obstacle_angle = math.atan2(obs_dy, obs_dx)
        
        # Compute velocity command
        cmd = Twist()
        
        if closest_distance < self.critical_dist:
            # CRITICAL: Stop and turn away
            cmd.linear.x = -0.1  # Back up slowly
            # Turn away from obstacle
            avoid_angle = self.normalize_angle(obstacle_angle - self.robot_yaw)
            if avoid_angle > 0:
                cmd.angular.z = -self.max_angular
            else:
                cmd.angular.z = self.max_angular
            self.get_logger().warn(f"CRITICAL: Obstacle at {closest_distance:.2f}m! Backing up...")
            
        elif closest_distance < self.obs_threshold:
            # OBSTACLE NEARBY: Navigate around it
            # Check if obstacle is in the direction of goal
            obs_angle_diff = self.normalize_angle(obstacle_angle - self.robot_yaw)
            
            if abs(obs_angle_diff) < math.pi/3:  # Obstacle is roughly ahead
                # Turn away from obstacle while moving
                if obs_angle_diff > 0:
                    cmd.angular.z = -self.max_angular * 0.8
                else:
                    cmd.angular.z = self.max_angular * 0.8
                # Reduce forward speed
                cmd.linear.x = self.max_linear * 0.3
                self.get_logger().info(
                    f"Avoiding obstacle at {closest_distance:.2f}m, turning..."
                )
            else:
                # Obstacle is to the side, proceed toward goal
                cmd.linear.x = self.max_linear * 0.5
                cmd.angular.z = angle_diff * 0.5
                
        else:
            # NO OBSTACLES: Navigate toward goal
            # Angular velocity based on angle to goal
            cmd.angular.z = np.clip(angle_diff * 1.0, -self.max_angular, self.max_angular)
            
            # Linear velocity based on alignment with goal
            if abs(angle_diff) < math.pi/4:  # Roughly facing goal
                cmd.linear.x = self.max_linear
            elif abs(angle_diff) < math.pi/2:
                cmd.linear.x = self.max_linear * 0.5
            else:
                # Turn in place if facing wrong direction
                cmd.linear.x = 0.0
        
        # Publish command
        self.cmd_pub.publish(cmd)
        
        # Log status periodically
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds > 2e9:  # Every 2 seconds
            self.get_logger().info(
                f"Nav: pos=({self.robot_x:.1f}, {self.robot_y:.1f}), "
                f"goal_dist={goal_distance:.1f}m, "
                f"obstacles={len(self.obstacles)}, "
                f"closest={closest_distance:.1f}m"
            )
            self.last_cmd_time = now
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def publish_stop(self):
        """Publish zero velocity command"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VisionNavigatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

















