#!/usr/bin/env python3
"""
Demo Point Cloud Node
Generates point clouds from camera images for visualization in RViz.
This is a simplified version that works without MUSt3R for demo purposes.

For Assignment 4 Part 2 demonstration.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct


class DemoPointCloudNode(Node):
    """
    Generates demo point clouds from camera images.
    Simulates 3D reconstruction for visualization.
    """

    def __init__(self):
        super().__init__('demo_pointcloud_node')

        self.get_logger().info("=" * 60)
        self.get_logger().info("Demo Point Cloud Node Started")
        self.get_logger().info("=" * 60)

        # Image storage
        self.base_image = None
        self.arm_image = None

        # Subscribers for camera images
        self.base_sub = self.create_subscription(
            Image,
            '/zed_base/left/image_raw',
            self.base_image_callback,
            10
        )

        self.arm_sub = self.create_subscription(
            Image,
            '/zed_arm/left/image_raw',
            self.arm_image_callback,
            10
        )

        # Point cloud publisher
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/must3r/point_cloud',
            10
        )

        # Timer for point cloud generation (2 Hz)
        self.timer = self.create_timer(0.5, self.generate_pointcloud)

        self.get_logger().info("Subscribed to: /zed_base/left/image_raw")
        self.get_logger().info("Subscribed to: /zed_arm/left/image_raw")
        self.get_logger().info("Publishing to: /must3r/point_cloud")
        self.get_logger().info("=" * 60)

    def base_image_callback(self, msg):
        """Store base camera image"""
        self.base_image = msg
        self.get_logger().debug(f"Base image: {msg.width}x{msg.height}")

    def arm_image_callback(self, msg):
        """Store arm camera image"""
        self.arm_image = msg
        self.get_logger().debug(f"Arm image: {msg.width}x{msg.height}")

    def generate_pointcloud(self):
        """Generate point cloud from camera images"""
        if self.base_image is None:
            self.get_logger().warn("Waiting for camera images...")
            return

        try:
            # Generate simulated 3D points from image
            points = self.image_to_points(self.base_image, "base")
            
            if self.arm_image is not None:
                arm_points = self.image_to_points(self.arm_image, "arm")
                points = np.vstack([points, arm_points])

            # Publish point cloud
            pc_msg = self.create_pointcloud2_msg(points, 'base_link')
            self.pointcloud_pub.publish(pc_msg)

            self.get_logger().info(f"Published point cloud: {len(points)} points")

        except Exception as e:
            self.get_logger().error(f"Error generating point cloud: {e}")

    def image_to_points(self, image_msg, source):
        """
        Convert image to 3D points using depth estimation.
        This is a simplified simulation for demo purposes.
        """
        # Decode image
        width = image_msg.width
        height = image_msg.height
        
        # Sample points from image (downsample for performance)
        step = 16  # Sample every 16th pixel
        num_h = height // step
        num_w = width // step
        
        points = []
        
        # Generate 3D points
        for i in range(num_h):
            for j in range(num_w):
                # Pixel coordinates
                px = j * step
                py = i * step
                
                # Simulated depth (varies with vertical position)
                depth = 2.0 + (i / num_h) * 3.0  # 2-5 meters
                
                # Add some noise for realism
                depth += np.random.randn() * 0.1
                
                # Convert to 3D (simple pinhole camera model)
                fx = fy = 500  # Focal length
                cx, cy = width / 2, height / 2
                
                x = (px - cx) * depth / fx
                y = (py - cy) * depth / fy
                z = depth
                
                # Offset based on camera source
                if source == "arm":
                    x += 0.5  # Arm camera offset
                    z += 0.5
                
                # Get color from image
                idx = (py * width + px) * 3
                if idx + 2 < len(image_msg.data):
                    r = image_msg.data[idx]
                    g = image_msg.data[idx + 1]
                    b = image_msg.data[idx + 2]
                else:
                    r = g = b = 128
                
                points.append([x, y, z, r, g, b])
        
        return np.array(points)

    def create_pointcloud2_msg(self, points, frame_id):
        """Create PointCloud2 message with RGB colors"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        point_struct = struct.Struct('fffI')
        buffer = bytearray(point_struct.size * len(points))

        for i, point in enumerate(points):
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            r, g, b = int(point[3]), int(point[4]), int(point[5])
            rgb = (r << 16) | (g << 8) | b
            point_struct.pack_into(buffer, i * point_struct.size, x, y, z, rgb)

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_struct.size
        msg.row_step = point_struct.size * len(points)
        msg.is_dense = True
        msg.data = bytes(buffer)

        return msg


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DemoPointCloudNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

















