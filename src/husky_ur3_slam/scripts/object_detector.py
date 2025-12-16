#!/usr/bin/env python3
"""
Object Detector Node
Assignment 4 Part 2 - Integration Demo

Detects objects using laser scanner data through simple clustering.
Publishes detected object poses for manipulation.

Algorithm:
1. Read laser scan data
2. Cluster nearby points
3. Calculate cluster centroids
4. Publish as object poses
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import math

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # Parameters
        self.declare_parameter('min_cluster_size', 5)
        self.declare_parameter('max_cluster_size', 50)
        self.declare_parameter('cluster_distance_threshold', 0.2)  # 20cm
        self.declare_parameter('min_object_distance', 0.5)  # 50cm from robot
        self.declare_parameter('max_object_distance', 3.0)  # 3m from robot

        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value
        self.cluster_threshold = self.get_parameter('cluster_distance_threshold').value
        self.min_distance = self.get_parameter('min_object_distance').value
        self.max_distance = self.get_parameter('max_object_distance').value

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publishers
        self.objects_pub = self.create_publisher(
            PoseArray,
            '/detected_objects',
            10
        )

        self.get_logger().info('Object Detector initialized')
        self.get_logger().info(f'Cluster threshold: {self.cluster_threshold}m')
        self.get_logger().info(f'Detection range: {self.min_distance}m - {self.max_distance}m')

    def scan_callback(self, scan_msg):
        """Process laser scan and detect objects"""
        # Convert laser scan to cartesian points
        points = self.scan_to_points(scan_msg)

        if len(points) == 0:
            return

        # Cluster points
        clusters = self.cluster_points(points)

        if len(clusters) == 0:
            return

        # Convert clusters to object poses
        objects = PoseArray()
        objects.header = scan_msg.header
        objects.header.frame_id = 'base_link'

        for cluster in clusters:
            # Calculate centroid
            centroid_x = np.mean([p[0] for p in cluster])
            centroid_y = np.mean([p[1] for p in cluster])

            # Create pose
            pose = Pose()
            pose.position.x = centroid_x
            pose.position.y = centroid_y
            pose.position.z = 0.0
            pose.orientation.w = 1.0

            objects.poses.append(pose)

        # Publish detected objects
        if len(objects.poses) > 0:
            self.objects_pub.publish(objects)
            self.get_logger().debug(f'Detected {len(objects.poses)} objects')

    def scan_to_points(self, scan_msg):
        """Convert laser scan to cartesian points"""
        points = []

        angle = scan_msg.angle_min
        for i, range_val in enumerate(scan_msg.ranges):
            # Filter invalid and out-of-range points
            if (range_val < scan_msg.range_min or
                range_val > scan_msg.range_max or
                range_val < self.min_distance or
                range_val > self.max_distance or
                math.isnan(range_val) or
                math.isinf(range_val)):
                angle += scan_msg.angle_increment
                continue

            # Convert to cartesian coordinates
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)

            points.append((x, y))

            angle += scan_msg.angle_increment

        return points

    def cluster_points(self, points):
        """Cluster points using simple distance-based clustering"""
        if len(points) == 0:
            return []

        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            # Calculate distance to last point in current cluster
            dist = math.sqrt(
                (points[i][0] - current_cluster[-1][0])**2 +
                (points[i][1] - current_cluster[-1][1])**2
            )

            if dist < self.cluster_threshold:
                # Add to current cluster
                current_cluster.append(points[i])
            else:
                # Save current cluster if it's valid size
                if self.min_cluster_size <= len(current_cluster) <= self.max_cluster_size:
                    clusters.append(current_cluster)

                # Start new cluster
                current_cluster = [points[i]]

        # Don't forget the last cluster
        if self.min_cluster_size <= len(current_cluster) <= self.max_cluster_size:
            clusters.append(current_cluster)

        return clusters


def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetector()

    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
