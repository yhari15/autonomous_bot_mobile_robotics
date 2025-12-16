#!/usr/bin/env python3
"""
Arm Trajectory Bridge Node
Provides FollowJointTrajectory action server for the UR3 arm
Translates trajectory commands to Gazebo Ignition joint position commands
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading
import time


class ArmTrajectoryBridge(Node):
    """Bridge between FollowJointTrajectory action and Gazebo Ignition joint position topics."""

    def __init__(self):
        super().__init__('arm_trajectory_bridge')

        # Joint names for UR3 arm (must match URDF and SRDF)
        self.joint_names = [
            'ur3_joint1',
            'ur3_joint2',
            'ur3_joint3',
            'ur3_joint4',
            'ur3_joint5',
            'ur3_joint6'
        ]

        # Current joint positions (updated from /joint_states)
        self.current_positions = {joint: 0.0 for joint in self.joint_names}
        self.joint_states_received = False

        # Subscribe to joint states to get current positions
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers for Gazebo Ignition joint position commands
        # Using custom topic names defined in URDF: /ur3_joint1_cmd, etc.
        self.joint_cmd_pubs = {}
        for joint in self.joint_names:
            topic = f'/{joint}_cmd'
            self.joint_cmd_pubs[joint] = self.create_publisher(Float64, topic, 10)
            self.get_logger().info(f'Created publisher for {topic}')

        # Action server for FollowJointTrajectory
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('Arm Trajectory Bridge initialized')
        self.get_logger().info('Action server: /arm_controller/follow_joint_trajectory')
        self.get_logger().info('Publishing to Ignition joint position controllers')
        self.get_logger().info('=' * 60)

        # Lock for thread-safe goal handling
        self._goal_lock = threading.Lock()
        self._current_goal = None

    def joint_state_callback(self, msg: JointState):
        """Update current joint positions from /joint_states topic."""
        for i, name in enumerate(msg.name):
            if name in self.current_positions and i < len(msg.position):
                self.current_positions[name] = msg.position[i]
        
        if not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info('Receiving joint states from Gazebo')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received trajectory goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the FollowJointTrajectory action."""
        self.get_logger().info('Executing trajectory...')

        with self._goal_lock:
            self._current_goal = goal_handle

        trajectory = goal_handle.request.trajectory
        joint_names = list(trajectory.joint_names)
        points = trajectory.points

        if not points:
            self.get_logger().warn('Received empty trajectory')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result

        # Validate joint names
        for joint in joint_names:
            if joint not in self.joint_names:
                self.get_logger().error(f'Unknown joint: {joint}')
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                return result

        self.get_logger().info(f'Trajectory has {len(points)} points for joints: {joint_names}')

        # Execute each trajectory point
        prev_time = 0.0
        for point_idx, point in enumerate(points):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Trajectory canceled')
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            # Calculate wait time for this point
            point_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            wait_time = max(0.0, point_time - prev_time)
            prev_time = point_time

            # Wait until it's time for this point
            if wait_time > 0.0:
                time.sleep(wait_time)

            # Command each joint to its target position
            for i, joint_name in enumerate(joint_names):
                if i < len(point.positions):
                    position = point.positions[i]
                    msg = Float64()
                    msg.data = float(position)
                    self.joint_cmd_pubs[joint_name].publish(msg)

            # Log progress
            if len(points) > 1:
                self.get_logger().info(f'Executed point {point_idx + 1}/{len(points)}')

            # Publish feedback
            feedback_msg = FollowJointTrajectory.Feedback()
            feedback_msg.header.stamp = self.get_clock().now().to_msg()
            feedback_msg.joint_names = joint_names
            feedback_msg.desired.positions = list(point.positions)
            actual_positions = [self.current_positions.get(j, 0.0) for j in joint_names]
            feedback_msg.actual.positions = actual_positions
            goal_handle.publish_feedback(feedback_msg)

        # Final point reached - wait a bit for joints to settle
        time.sleep(0.5)

        # Trajectory completed successfully
        goal_handle.succeed()
        self.get_logger().info('Trajectory completed successfully')

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL

        with self._goal_lock:
            self._current_goal = None

        return result


def main(args=None):
    rclpy.init(args=args)

    arm_bridge = ArmTrajectoryBridge()

    # Use MultiThreadedExecutor for action server
    executor = MultiThreadedExecutor()
    executor.add_node(arm_bridge)

    try:
        executor.spin()
    except KeyboardInterrupt:
        arm_bridge.get_logger().info('Shutting down Arm Trajectory Bridge')
    finally:
        arm_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
