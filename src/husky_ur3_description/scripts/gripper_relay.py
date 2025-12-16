#!/usr/bin/python3
"""
Gripper Relay Node - Bridges ROS 2 and Ignition Gazebo for gripper control

This node relays gripper commands from a ROS 2 topic to Gazebo's JointPositionController
which uses topic names that violate ROS 2 naming conventions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import subprocess


class GripperRelay(Node):
    def __init__(self):
        super().__init__('gripper_relay')

        # Subscribe to ROS 2 gripper command topic
        self.subscription = self.create_subscription(
            Float64,
            '/gripper_cmd',
            self.gripper_callback,
            10
        )

        self.get_logger().info('Gripper relay node started - listening on /gripper_cmd')

    def gripper_callback(self, msg):
        """Relay gripper command to Gazebo via ign topic"""
        try:
            # Clamp value to valid range [0.0, 0.8]
            value = max(0.0, min(0.8, msg.data))

            # Publish to Gazebo topic using ign command
            cmd = [
                'ign', 'topic', '-t',
                '/model/husky_ur3/joint/finger_joint/0/cmd_pos',
                '-m', 'ignition.msgs.Double',
                '-p', f'data: {value}',
                '-n', '1'
            ]

            subprocess.run(cmd, check=True, capture_output=True)
            self.get_logger().info(f'Sent gripper command: {value:.2f}')

        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to send gripper command: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in gripper callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GripperRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
