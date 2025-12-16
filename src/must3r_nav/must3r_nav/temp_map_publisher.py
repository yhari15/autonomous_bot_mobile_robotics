#!/usr/bin/env python3
"""
Temporary Map Frame Publisher
Publishes a static map->odom transform until SLAM initializes
This prevents TF errors during startup
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import time


class TempMapPublisher(Node):
    """Publishes temporary map->odom transform until SLAM takes over"""

    def __init__(self):
        super().__init__('temp_map_publisher')

        # Static transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Check if map frame already exists
        self.timer = self.create_timer(1.0, self.check_and_publish)

        self.get_logger().info('Temporary map frame publisher started')
        self.get_logger().info('Will publish map->odom until SLAM initializes')

        # Track if we're publishing
        self.publishing = False
        self.slam_active = False

    def check_and_publish(self):
        """Check if SLAM is publishing map frame, if not publish temporary one"""

        # Check if /map topic exists (SLAM is publishing)
        topic_list = self.get_topic_names_and_types()
        map_exists = any('/map' in topic[0] for topic in topic_list)

        if map_exists and not self.slam_active:
            self.get_logger().info('SLAM is now publishing map! Stopping temporary transform.')
            self.slam_active = True
            # Destroy timer to stop checking
            self.timer.cancel()
            return

        if not map_exists and not self.publishing:
            self.get_logger().info('Map frame not yet available, publishing temporary transform')
            self.publish_static_transform()
            self.publishing = True

    def publish_static_transform(self):
        """Publish static map->odom transform"""
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # Identity transform (map and odom aligned at start)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published static map->odom transform')


def main(args=None):
    rclpy.init(args=args)

    node = TempMapPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
