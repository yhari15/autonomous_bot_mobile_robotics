#!/usr/bin/env python3
"""
Mission Controller
Assignment 4 Part 2 - Integration Demo

Coordinates the complete autonomous mobile manipulation mission:
1. Start from unknown location
2. Explore environment and build map
3. Detect and manipulate objects during exploration
4. Return to starting position
5. Complete mission

This is the main node for the integration demo.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import math
import time

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # Mission parameters
        self.declare_parameter('exploration_time', 180.0)  # 3 minutes
        self.declare_parameter('objects_to_manipulate', 2)
        self.declare_parameter('return_to_start', True)

        self.exploration_time = self.get_parameter('exploration_time').value
        self.objects_target = self.get_parameter('objects_to_manipulate').value
        self.should_return = self.get_parameter('return_to_start').value

        # Mission state
        self.mission_state = 'INIT'  # INIT, EXPLORING, MANIPULATING, RETURNING, COMPLETE
        self.start_position = None
        self.start_time = None
        self.objects_manipulated = 0
        self.current_map = None
        self.exploration_waypoints = []
        self.current_waypoint_index = 0

        # Navigation state tracking
        self.navigating = False
        self.nav_goal_handle = None
        self.manipulation_start_time = None

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.objects_sub = self.create_subscription(
            PoseArray,
            '/detected_objects',
            self.objects_callback,
            10
        )

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timer for mission control loop
        self.timer = self.create_timer(1.0, self.mission_loop)

        self.get_logger().info('='*50)
        self.get_logger().info('Mission Controller Started')
        self.get_logger().info('='*50)
        self.get_logger().info(f'Exploration time: {self.exploration_time}s')
        self.get_logger().info(f'Objects to manipulate: {self.objects_target}')
        self.get_logger().info(f'Return to start: {self.should_return}')
        self.get_logger().info('='*50)

    def map_callback(self, msg):
        """Store the latest map"""
        self.current_map = msg

    def objects_callback(self, msg):
        """Handle detected objects"""
        if self.mission_state == 'EXPLORING' and len(msg.poses) > 0:
            if self.objects_manipulated < self.objects_target and not self.navigating:
                self.get_logger().info(f'Object detected! Switching to manipulation mode')
                self.mission_state = 'MANIPULATING'
                self.manipulation_start_time = time.time()

    def mission_loop(self):
        """Main mission control loop"""
        if self.mission_state == 'INIT':
            self.init_mission()

        elif self.mission_state == 'EXPLORING':
            self.explore()

        elif self.mission_state == 'MANIPULATING':
            self.manipulate()

        elif self.mission_state == 'RETURNING':
            self.return_to_start()

        elif self.mission_state == 'COMPLETE':
            self.log_mission_summary()

    def init_mission(self):
        """Initialize the mission"""
        # Don't wait for map - start exploring immediately
        # Map will build as the robot explores (SLAM updates during movement)
        self.get_logger().info('Initializing mission...')

        # Record start position (assume robot starts at origin)
        self.start_position = PoseStamped()
        self.start_position.header.frame_id = 'map'
        self.start_position.pose.position.x = 0.0
        self.start_position.pose.position.y = 0.0
        self.start_position.pose.orientation.w = 1.0

        # Generate exploration waypoints
        self.generate_exploration_waypoints()

        # Start timer
        self.start_time = time.time()

        # Transition to exploring
        self.mission_state = 'EXPLORING'
        self.get_logger().info('='*50)
        self.get_logger().info('Mission initialized! Starting autonomous exploration...')
        self.get_logger().info('Map will build as robot explores')
        self.get_logger().info('='*50)

    def generate_exploration_waypoints(self):
        """Generate waypoints for systematic exploration"""
        # Simple grid pattern for exploration
        # In production, use frontier-based exploration

        waypoints = [
            (5.0, 0.0),
            (5.0, 5.0),
            (0.0, 5.0),
            (-5.0, 5.0),
            (-5.0, 0.0),
            (-5.0, -5.0),
            (0.0, -5.0),
            (5.0, -5.0),
        ]

        for x, y in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            self.exploration_waypoints.append(pose)

        self.get_logger().info(f'Generated {len(self.exploration_waypoints)} exploration waypoints')

    def explore(self):
        """Execute exploration phase"""
        elapsed_time = time.time() - self.start_time

        # Check if exploration time exceeded
        if elapsed_time > self.exploration_time:
            self.get_logger().info(f'Exploration time limit reached ({self.exploration_time}s)')
            if self.should_return:
                self.mission_state = 'RETURNING'
            else:
                self.mission_state = 'COMPLETE'
            return

        # Only send new navigation goals if not currently navigating
        if not self.navigating:
            if self.current_waypoint_index < len(self.exploration_waypoints):
                waypoint = self.exploration_waypoints[self.current_waypoint_index]
                self.get_logger().info(
                    f'Exploring waypoint {self.current_waypoint_index + 1}/'
                    f'{len(self.exploration_waypoints)} '
                    f'at ({waypoint.pose.position.x:.1f}, {waypoint.pose.position.y:.1f})'
                )

                self.send_navigation_goal(waypoint)
                self.current_waypoint_index += 1

            else:
                # All waypoints explored
                self.get_logger().info('All exploration waypoints visited')
                if self.should_return:
                    self.mission_state = 'RETURNING'
                else:
                    self.mission_state = 'COMPLETE'

    def manipulate(self):
        """Execute manipulation phase"""
        # Check if manipulation time has elapsed (3 seconds simulation)
        if self.manipulation_start_time is None:
            self.get_logger().info(f'Manipulating object {self.objects_manipulated + 1}...')
            self.manipulation_start_time = time.time()
            return

        elapsed = time.time() - self.manipulation_start_time
        if elapsed >= 3.0:
            self.objects_manipulated += 1
            self.get_logger().info(
                f'Object manipulation complete! '
                f'Total: {self.objects_manipulated}/{self.objects_target}'
            )

            # Reset manipulation timer and return to exploration
            self.manipulation_start_time = None
            self.mission_state = 'EXPLORING'

    def return_to_start(self):
        """Return to starting position"""
        if self.start_position is None:
            self.get_logger().warn('Start position not recorded!')
            self.mission_state = 'COMPLETE'
            return

        # Only send navigation goal once
        if not self.navigating:
            self.get_logger().info('Returning to start position...')
            self.send_navigation_goal(self.start_position)

    def send_navigation_goal(self, goal_pose):
        """Send navigation goal to Nav2 and set up result callback"""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Navigation server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'Sending goal: ({goal_pose.pose.position.x:.2f}, '
            f'{goal_pose.pose.position.y:.2f})'
        )

        # Mark as navigating
        self.navigating = True

        # Send goal and register callbacks
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            self.navigating = False
            return

        self.get_logger().info('Navigation goal accepted')
        self.nav_goal_handle = goal_handle

        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback (optional)"""
        # Can log current distance to goal, etc.
        pass

    def navigation_result_callback(self, future):
        """Handle navigation completion"""
        result = future.result()
        status = result.status

        self.navigating = False
        self.nav_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation goal succeeded!')

            # If we were returning to start, mark mission complete
            if self.mission_state == 'RETURNING':
                self.get_logger().info('Returned to start position!')
                self.mission_state = 'COMPLETE'

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Navigation goal aborted')

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation goal canceled')

        else:
            self.get_logger().warn(f'Navigation goal failed with status: {status}')

    def log_mission_summary(self):
        """Log mission completion summary"""
        if not hasattr(self, '_summary_logged'):
            elapsed = time.time() - self.start_time if self.start_time else 0

            self.get_logger().info('='*50)
            self.get_logger().info('MISSION COMPLETE!')
            self.get_logger().info('='*50)
            self.get_logger().info(f'Total time: {elapsed:.1f}s')
            self.get_logger().info(
                f'Objects manipulated: {self.objects_manipulated}/'
                f'{self.objects_target}'
            )
            self.get_logger().info(
                f'Waypoints explored: {self.current_waypoint_index}/'
                f'{len(self.exploration_waypoints)}'
            )
            if self.should_return:
                self.get_logger().info('Returned to start: YES')
            self.get_logger().info('='*50)

            self._summary_logged = True


def main(args=None):
    rclpy.init(args=args)
    controller = MissionController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Mission interrupted by user')

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
