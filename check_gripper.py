#!/usr/bin/env python3
"""
Check gripper joint state and test gripper commands.
Usage: python3 check_gripper.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class GripperChecker(Node):
    def __init__(self):
        super().__init__('gripper_checker')
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # Publisher for gripper command
        self.gripper_pub = self.create_publisher(
            Float64, '/rh_p12_rn_position/command', 10)
        
        self.gripper_position = None
        self.get_logger().info('Gripper checker started')
        
    def joint_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if 'rh_p12_rn' in name or 'gripper' in name.lower():
                self.gripper_position = pos
                self.get_logger().info(f'Gripper joint: {name} = {pos:.4f}')
    
    def open_gripper(self):
        """Open the gripper."""
        msg = Float64()
        msg.data = 0.0
        self.gripper_pub.publish(msg)
        self.get_logger().info('Sent OPEN command (0.0)')
    
    def close_gripper(self):
        """Close the gripper."""
        msg = Float64()
        msg.data = 1.0
        self.gripper_pub.publish(msg)
        self.get_logger().info('Sent CLOSE command (1.0)')


def main():
    rclpy.init()
    checker = GripperChecker()
    
    print("\nGripper Checker")
    print("Commands: 'o' = open, 'c' = close, 'q' = quit")
    print("Listening for joint states...\n")
    
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(checker,), daemon=True)
    spin_thread.start()
    
    try:
        while True:
            cmd = input("Enter command (o/c/q): ").strip().lower()
            if cmd == 'o':
                checker.open_gripper()
            elif cmd == 'c':
                checker.close_gripper()
            elif cmd == 'q':
                break
    except KeyboardInterrupt:
        pass
    
    checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

