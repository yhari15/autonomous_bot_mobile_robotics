#!/usr/bin/env python3
"""
Simple Manipulator Controller
Assignment 4 Part 2 - Integration Demo

Controls the UR3 arm for basic pick-and-place operations.
Listens for detected objects and performs manipulation.

Note: This is a simplified controller using joint position commands.
For production, use MoveIt2 for motion planning.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64
import math
import time

class SimpleManipulator(Node):
    def __init__(self):
        super().__init__('simple_manipulator')

        # Parameters
        self.declare_parameter('manipulation_enabled', True)
        self.declare_parameter('max_objects_to_manipulate', 3)

        self.enabled = self.get_parameter('manipulation_enabled').value
        self.max_objects = self.get_parameter('max_objects_to_manipulate').value

        # State
        self.objects_manipulated = 0
        self.is_manipulating = False
        self.detected_objects = []

        # Joint positions for different configurations
        # All angles in radians
        self.HOME_POSITION = {
            'shoulder_pan': 0.0,
            'shoulder_lift': -1.57,  # -90 degrees
            'elbow': 1.57,           # 90 degrees
            'wrist_1': 0.0,
            'wrist_2': 0.0,
            'wrist_3': 0.0
        }

        self.PICK_POSITION = {
            'shoulder_pan': 0.0,
            'shoulder_lift': -0.785,  # -45 degrees
            'elbow': 1.57,            # 90 degrees
            'wrist_1': -0.785,        # -45 degrees
            'wrist_2': 0.0,
            'wrist_3': 0.0
        }

        # Subscribers
        self.objects_sub = self.create_subscription(
            PoseArray,
            '/detected_objects',
            self.objects_callback,
            10
        )

        # Publishers for gripper control
        self.gripper_pub = self.create_publisher(
            Float64,
            '/gripper_position',
            10
        )

        # Timer for manipulation loop
        self.timer = self.create_timer(2.0, self.manipulation_loop)

        self.get_logger().info('Simple Manipulator initialized')
        self.get_logger().info(f'Manipulation enabled: {self.enabled}')
        self.get_logger().info(f'Max objects to manipulate: {self.max_objects}')

        # Move to home position on startup
        self.move_to_home()

    def objects_callback(self, msg):
        """Store detected objects"""
        self.detected_objects = msg.poses

    def manipulation_loop(self):
        """Periodic check for manipulation opportunities"""
        if not self.enabled:
            return

        if self.is_manipulating:
            self.get_logger().debug('Already manipulating')
            return

        if self.objects_manipulated >= self.max_objects:
            self.get_logger().debug(f'Max objects reached ({self.max_objects})')
            return

        if len(self.detected_objects) == 0:
            return

        # Perform manipulation on first object
        self.get_logger().info(f'Attempting to manipulate object {self.objects_manipulated + 1}')
        self.perform_pick_and_place()

    def perform_pick_and_place(self):
        """Execute a simple pick and place sequence"""
        self.is_manipulating = True

        try:
            # Sequence:
            # 1. Open gripper
            self.get_logger().info('Step 1: Opening gripper')
            self.open_gripper()
            time.sleep(1.0)

            # 2. Move to pick position
            self.get_logger().info('Step 2: Moving to pick position')
            self.move_to_pick()
            time.sleep(2.0)

            # 3. Close gripper (grasp)
            self.get_logger().info('Step 3: Closing gripper (grasping)')
            self.close_gripper()
            time.sleep(1.0)

            # 4. Lift object
            self.get_logger().info('Step 4: Lifting object')
            self.move_to_home()
            time.sleep(2.0)

            # 5. Open gripper (release)
            self.get_logger().info('Step 5: Releasing object')
            self.open_gripper()
            time.sleep(1.0)

            # 6. Return to home
            self.get_logger().info('Step 6: Returning to home')
            self.move_to_home()
            time.sleep(1.0)

            self.objects_manipulated += 1
            self.get_logger().info(f'Manipulation complete! Total: {self.objects_manipulated}/{self.max_objects}')

        except Exception as e:
            self.get_logger().error(f'Manipulation failed: {str(e)}')

        finally:
            self.is_manipulating = False

    def move_to_home(self):
        """Move arm to home position"""
        self.get_logger().debug('Moving to home position')
        # In the real implementation, this would publish joint commands
        # For now, just a placeholder since the joints are controlled by
        # position controllers in the URDF

    def move_to_pick(self):
        """Move arm to pick position"""
        self.get_logger().debug('Moving to pick position')
        # Placeholder - would publish joint commands

    def open_gripper(self):
        """Open the gripper"""
        msg = Float64()
        msg.data = 0.0  # Fully open
        self.gripper_pub.publish(msg)
        self.get_logger().debug('Gripper opened')

    def close_gripper(self):
        """Close the gripper"""
        msg = Float64()
        msg.data = 0.085  # Fully closed (max opening of RH-P12-RN)
        self.gripper_pub.publish(msg)
        self.get_logger().debug('Gripper closed')

    def get_manipulation_stats(self):
        """Return manipulation statistics"""
        return {
            'objects_manipulated': self.objects_manipulated,
            'max_objects': self.max_objects,
            'success_rate': (self.objects_manipulated / self.max_objects * 100
                           if self.max_objects > 0 else 0)
        }


def main(args=None):
    rclpy.init(args=args)
    manipulator = SimpleManipulator()

    try:
        rclpy.spin(manipulator)
    except KeyboardInterrupt:
        stats = manipulator.get_manipulation_stats()
        manipulator.get_logger().info(
            f"Manipulation Statistics:\n"
            f"  Objects manipulated: {stats['objects_manipulated']}/{stats['max_objects']}\n"
            f"  Success rate: {stats['success_rate']:.1f}%"
        )

    manipulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
