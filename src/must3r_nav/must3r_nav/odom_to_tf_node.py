#!/usr/bin/env python3
"""
Odometry to TF Publisher Node

Subscribes to /odom and publishes the odom -> base_link transform.
This is needed because Gazebo Ignition doesn't have a native TF publisher
and the ros_gz_bridge doesn't properly convert Pose_V to TF.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTFNode(Node):
    """Publishes odom -> base_link transform from /odom topic."""

    def __init__(self):
        super().__init__('odom_to_tf_node')
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('OdomToTF node started - publishing odom->base_link TF')

    def odom_callback(self, msg: Odometry):
        """Convert odometry message to TF transform."""
        t = TransformStamped()
        
        # Header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # typically 'odom'
        t.child_frame_id = msg.child_frame_id    # typically 'base_link'
        
        # Translation from odometry pose
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Rotation from odometry pose
        t.transform.rotation = msg.pose.pose.orientation
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()










