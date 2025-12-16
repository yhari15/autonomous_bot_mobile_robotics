#!/usr/bin/env python3
"""
Simple Exploration Node
Assignment 4 Part 2 - Autonomous Exploration

This node performs basic frontier-based exploration:
1. Analyzes the SLAM map for frontiers (boundaries between known/unknown)
2. Selects the best frontier based on distance and information gain
3. Sends navigation goals to explore the environment

Note: This is a simplified version for demonstration.
For production, use explore_lite or other exploration packages.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
import math

class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')

        # Parameters
        self.declare_parameter('min_frontier_size', 10)
        self.declare_parameter('max_exploration_distance', 5.0)

        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.max_distance = self.get_parameter('max_exploration_distance').value

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # State
        self.current_map = None
        self.exploring = False
        self.current_goal = None

        self.get_logger().info('Simple Explorer initialized')
        self.get_logger().info(f'Min frontier size: {self.min_frontier_size}')
        self.get_logger().info(f'Max exploration distance: {self.max_distance}m')

        # Timer to periodically check for new frontiers
        self.timer = self.create_timer(5.0, self.explore_callback)

    def map_callback(self, msg):
        """Store the latest map"""
        self.current_map = msg

    def explore_callback(self):
        """Main exploration loop"""
        if self.current_map is None:
            self.get_logger().info('Waiting for map...')
            return

        if self.exploring:
            self.get_logger().debug('Already navigating to a goal')
            return

        # Find frontiers in the map
        frontiers = self.find_frontiers(self.current_map)

        if not frontiers:
            self.get_logger().info('No frontiers found - exploration complete!')
            return

        # Select best frontier
        best_frontier = self.select_best_frontier(frontiers)

        if best_frontier is None:
            self.get_logger().info('No suitable frontier found')
            return

        # Send navigation goal
        self.send_navigation_goal(best_frontier)

    def find_frontiers(self, occupancy_grid):
        """
        Find frontier cells (boundaries between free and unknown space)

        Returns: List of frontier points [(x, y), ...]
        """
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution
        origin = occupancy_grid.info.origin

        data = np.array(occupancy_grid.data).reshape((height, width))

        frontiers = []

        # Scan map for frontier cells
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                # Check if cell is free (0)
                if data[y, x] == 0:
                    # Check if adjacent to unknown (-1)
                    if self.has_unknown_neighbor(data, x, y):
                        # Convert grid coordinates to world coordinates
                        world_x = origin.position.x + (x + 0.5) * resolution
                        world_y = origin.position.y + (y + 0.5) * resolution
                        frontiers.append((world_x, world_y))

        self.get_logger().info(f'Found {len(frontiers)} frontier cells')
        return frontiers

    def has_unknown_neighbor(self, data, x, y):
        """Check if cell has at least one unknown neighbor"""
        neighbors = [
            data[y-1, x], data[y+1, x],  # Up, down
            data[y, x-1], data[y, x+1],  # Left, right
        ]
        return -1 in neighbors

    def select_best_frontier(self, frontiers):
        """
        Select the best frontier based on distance

        For now, just select the closest frontier.
        Could be enhanced with information gain, etc.
        """
        if not frontiers:
            return None

        # For simplicity, assume robot is at origin
        # In production, would get robot pose from TF
        robot_x, robot_y = 0.0, 0.0

        best_frontier = None
        min_distance = float('inf')

        for fx, fy in frontiers:
            distance = math.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)

            if distance < min_distance and distance < self.max_distance:
                min_distance = distance
                best_frontier = (fx, fy)

        if best_frontier:
            self.get_logger().info(f'Selected frontier at ({best_frontier[0]:.2f}, {best_frontier[1]:.2f}), distance: {min_distance:.2f}m')

        return best_frontier

    def send_navigation_goal(self, frontier):
        """Send navigation goal to Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = frontier[0]
        goal_msg.pose.pose.position.y = frontier[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending navigation goal: ({frontier[0]:.2f}, {frontier[1]:.2f})')

        # Send goal
        self.exploring = True
        self.current_goal = frontier

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected')
            self.exploring = False
            return

        self.get_logger().info('Goal accepted, navigating...')

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.exploring = False

        self.get_logger().info('Navigation complete, searching for next frontier...')


def main(args=None):
    rclpy.init(args=args)
    explorer = SimpleExplorer()

    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass

    explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
