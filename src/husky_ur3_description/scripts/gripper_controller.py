#!/usr/bin/env python3
"""
Gripper Controller Node
Automatically opens and closes the gripper with proper timing for demonstration
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # Publisher for Robotis RH-P12-RN gripper commands
        self.publisher_ = self.create_publisher(
            Float64, 
            '/rh_p12_rn_position/command', 
            10
        )
        
        # State tracking
        self.is_open = False  # Start FALSE so first command is OPEN (0.0)
        self.current_target = 0.0  # Start at open position
        
        # High-frequency publisher (20 Hz for continuous commands)
        self.publish_timer = self.create_timer(0.05, self.publish_callback)
        
        # Toggle timer (15 seconds for much slower demonstration - gives time to fully complete movement)
        self.toggle_timer = self.create_timer(15.0, self.toggle_callback)
        
        self.get_logger().info('='*60)
        self.get_logger().info('🤖 Robotis RH-P12-RN Gripper Controller Started!')
        self.get_logger().info('='*60)
        self.get_logger().info('📊 Configuration:')
        self.get_logger().info('  - Toggle interval: 15 seconds (gives time to fully move)')
        self.get_logger().info('  - Publishing rate: 20 Hz (continuous commands)')
        self.get_logger().info('  - Open position: 0.0 rad')
        self.get_logger().info('  - Close position: 0.65 rad')
        self.get_logger().info('  - Starting position: OPEN (0.0 rad)')
        self.get_logger().info('='*60)

    def publish_callback(self):
        """Continuously publish current target at 20 Hz"""
        msg = Float64()
        msg.data = self.current_target
        self.publisher_.publish(msg)

    def toggle_callback(self):
        """Toggle gripper position every 8 seconds"""
        if self.is_open:
            self.current_target = 0.65  # Close gripper (previous working value)
            self.get_logger().info('')
            self.get_logger().info('🔴'*20)
            self.get_logger().info('>>> CLOSING GRIPPER to 0.65 rad <<<')
            self.get_logger().info('🔴'*20)
        else:
            self.current_target = 0.0   # Open gripper
            self.get_logger().info('')
            self.get_logger().info('🟢'*20)
            self.get_logger().info('>>> OPENING GRIPPER to 0.0 rad <<<')
            self.get_logger().info('🟢'*20)
        
        self.is_open = not self.is_open
        self.get_logger().info(f'⏱️  Next toggle in 15 seconds...')
        self.get_logger().info('-'*60)


def main(args=None):
    rclpy.init(args=args)
    
    gripper_controller = GripperController()
    
    try:
        rclpy.spin(gripper_controller)
    except KeyboardInterrupt:
        gripper_controller.get_logger().info('Gripper controller stopped by user')
    finally:
        gripper_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
