#!/usr/bin/env python3
"""
Test script to send a navigation goal to Nav2.
Usage: python3 test_nav_goal.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math


class NavGoalTester(Node):
    def __init__(self):
        super().__init__('nav_goal_tester')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def send_goal(self, x, y, yaw=0.0):
        """Send a navigation goal."""
        self.get_logger().info(f'Waiting for Nav2 action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            return False
            
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        
        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw}')
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
            
        self.get_logger().info('Goal accepted! Waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        self.get_logger().info(f'Navigation result: {result.status}')
        return result.status == 4  # SUCCEEDED


def main():
    rclpy.init()
    tester = NavGoalTester()
    
    # Test goal - near the table
    success = tester.send_goal(x=3.0, y=1.5, yaw=-1.57)
    
    if success:
        print("Navigation succeeded!")
    else:
        print("Navigation failed!")
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

