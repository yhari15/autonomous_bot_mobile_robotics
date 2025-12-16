#!/usr/bin/env python3
"""
Stereo Depth Node - Simple stereo matching for 3D reconstruction
Works as a fallback if MUSt3R is not available.

Uses OpenCV's stereo matching to create point clouds from ZED camera images.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import numpy as np
import struct
import cv2
from pathlib import Path


class StereoDepthNode(Node):
    """Simple stereo depth node for 3D reconstruction."""

    def __init__(self):
        super().__init__('stereo_depth_node')

        self.get_logger().info("=" * 60)
        self.get_logger().info("Stereo Depth Node Started")
        self.get_logger().info("Using OpenCV StereoSGBM for depth estimation")
        self.get_logger().info("=" * 60)

        # CV Bridge
        self.bridge = CvBridge()

        # Camera parameters (ZED 2 defaults - will be updated from CameraInfo)
        self.baseline = 0.12  # 120mm baseline
        self.focal_length = 500.0  # Will be updated from camera_info

        # Image storage - base camera
        self.base_left = None
        self.base_right = None
        self.base_camera_info = None

        # Image storage - arm camera
        self.arm_left = None
        self.arm_right = None
        self.arm_camera_info = None

        # Point cloud accumulation
        self.accumulated_points = []
        self.frame_count = 0

        # Output directory
        self.pcd_dir = Path.home() / "ros2_ws" / "pointclouds"
        self.pcd_dir.mkdir(parents=True, exist_ok=True)

        # Stereo matcher (SGBM for better quality)
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,  # must be divisible by 16
            blockSize=11,
            P1=8 * 3 * 11**2,
            P2=32 * 3 * 11**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        # Subscribers - Base camera
        self.base_left_sub = self.create_subscription(
            Image, '/zed_base/left/image_raw', self.base_left_cb, 10)
        self.base_right_sub = self.create_subscription(
            Image, '/zed_base/right/image_raw', self.base_right_cb, 10)
        self.base_camera_info_sub = self.create_subscription(
            CameraInfo, '/zed_base/left/camera_info', self.base_camera_info_cb, 10)

        # Subscribers - Arm camera
        self.arm_left_sub = self.create_subscription(
            Image, '/zed_arm/left/image_raw', self.arm_left_cb, 10)
        self.arm_right_sub = self.create_subscription(
            Image, '/zed_arm/right/image_raw', self.arm_right_cb, 10)
        self.arm_camera_info_sub = self.create_subscription(
            CameraInfo, '/zed_arm/left/camera_info', self.arm_camera_info_cb, 10)

        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, '/stereo/point_cloud', 10)
        self.depth_pub = self.create_publisher(Image, '/stereo/depth', 10)

        # Services
        self.save_pcd_service = self.create_service(
            Trigger, '/must3r/save_pcd', self.save_pcd_callback)
        self.clear_points_service = self.create_service(
            Trigger, '/must3r/clear_points', self.clear_points_callback)

        # Timer for processing (2 Hz)
        self.process_timer = self.create_timer(0.5, self.process_stereo)

        self.get_logger().info(f"PCD output dir: {self.pcd_dir}")

    def base_left_cb(self, msg):
        self.base_left = msg

    def base_right_cb(self, msg):
        self.base_right = msg

    def base_camera_info_cb(self, msg):
        if self.base_camera_info is None:
            self.base_camera_info = msg
            self.focal_length = msg.k[0]
            self.get_logger().info(f"Base camera focal length: {self.focal_length}")

    def arm_left_cb(self, msg):
        self.arm_left = msg

    def arm_right_cb(self, msg):
        self.arm_right = msg

    def arm_camera_info_cb(self, msg):
        if self.arm_camera_info is None:
            self.arm_camera_info = msg
            self.get_logger().info(f"Arm camera focal length: {msg.k[0]}")

    def process_stereo(self):
        """Process stereo pairs from both cameras."""
        # Process base camera
        if self.base_left is not None and self.base_right is not None:
            points = self.compute_stereo_points(
                self.base_left, self.base_right,
                "zed_base_left_camera_optical_frame", "base"
            )
            if points is not None and len(points) > 0:
                self.accumulated_points.append(points)
            self.base_left = None
            self.base_right = None

        # Process arm camera
        if self.arm_left is not None and self.arm_right is not None:
            points = self.compute_stereo_points(
                self.arm_left, self.arm_right,
                "zed_arm_left_camera_optical_frame", "arm"
            )
            if points is not None and len(points) > 0:
                self.accumulated_points.append(points)
            self.arm_left = None
            self.arm_right = None

    def compute_stereo_points(self, left_msg, right_msg, frame_id, camera_name):
        """Compute 3D points from a stereo image pair."""
        try:
            # Convert to OpenCV format
            left_cv = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
            right_cv = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")

            # Convert to grayscale for stereo matching
            left_gray = cv2.cvtColor(left_cv, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_cv, cv2.COLOR_BGR2GRAY)

            # Compute disparity
            disparity = self.stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0

            # Filter invalid disparities
            valid_mask = disparity > 0

            if not np.any(valid_mask):
                return None

            # Compute depth from disparity: depth = baseline * focal / disparity
            depth = np.zeros_like(disparity)
            depth[valid_mask] = (self.baseline * self.focal_length) / disparity[valid_mask]

            # Filter by reasonable depth range (0.3m to 10m)
            valid_mask = (depth > 0.3) & (depth < 10.0)

            if not np.any(valid_mask):
                return None

            # Get image dimensions
            h, w = depth.shape

            # Create point cloud
            cx = w / 2.0
            cy = h / 2.0

            # Create coordinate grids
            u = np.arange(w)
            v = np.arange(h)
            u, v = np.meshgrid(u, v)

            # Compute 3D coordinates
            z = depth
            x = (u - cx) * z / self.focal_length
            y = (v - cy) * z / self.focal_length

            # Get colors from left image
            colors = left_cv

            # Stack into points array (N x 6: x, y, z, r, g, b)
            points_x = x[valid_mask]
            points_y = y[valid_mask]
            points_z = z[valid_mask]
            colors_b = colors[:, :, 0][valid_mask]
            colors_g = colors[:, :, 1][valid_mask]
            colors_r = colors[:, :, 2][valid_mask]

            points = np.column_stack([points_x, points_y, points_z, colors_r, colors_g, colors_b])

            # Downsample if too many points
            max_points = 30000
            if len(points) > max_points:
                indices = np.random.choice(len(points), max_points, replace=False)
                points = points[indices]

            self.frame_count += 1

            # Publish point cloud
            pc_msg = self.create_pc2_msg(points, frame_id)
            self.pc_pub.publish(pc_msg)

            # Publish depth image for visualization (base camera only)
            if camera_name == "base":
                depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                depth_uint8 = depth_normalized.astype(np.uint8)
                depth_msg = self.bridge.cv2_to_imgmsg(depth_uint8, "mono8")
                depth_msg.header = left_msg.header
                self.depth_pub.publish(depth_msg)

            if self.frame_count % 10 == 0:
                self.get_logger().info(
                    f"Frame {self.frame_count} ({camera_name}): {len(points)} pts, "
                    f"total: {sum(len(p) for p in self.accumulated_points)}"
                )

            return points

        except Exception as e:
            self.get_logger().error(f"Stereo processing error ({camera_name}): {e}")
            import traceback
            traceback.print_exc()
            return None

    def create_pc2_msg(self, points, frame_id):
        """Create PointCloud2 message."""
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
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            r = int(np.clip(point[3], 0, 255))
            g = int(np.clip(point[4], 0, 255))
            b = int(np.clip(point[5], 0, 255))
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

    def save_pcd_callback(self, request, response):
        """Save accumulated point cloud as PCD."""
        try:
            if not self.accumulated_points:
                response.success = False
                response.message = "No points accumulated"
                return response

            # Combine all points
            all_points = np.vstack(self.accumulated_points)

            # Remove duplicates by voxel grid downsampling
            voxel_size = 0.02  # 2cm voxels
            voxel_indices = np.floor(all_points[:, :3] / voxel_size).astype(int)
            _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
            all_points = all_points[unique_indices]

            # Save as PCD
            import time
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            pcd_path = self.pcd_dir / f"table_map_{timestamp}.pcd"

            self.save_pcd_file(all_points, pcd_path)

            response.success = True
            response.message = f"Saved {len(all_points)} points to {pcd_path}"
            self.get_logger().info(response.message)

            return response

        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            self.get_logger().error(response.message)
            return response

    def save_pcd_file(self, points, filepath):
        """Save points to ASCII PCD format."""
        with open(filepath, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z rgb\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F U\n")
            f.write("COUNT 1 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write("DATA ascii\n")

            for point in points:
                x, y, z = point[0], point[1], point[2]
                r = int(np.clip(point[3], 0, 255))
                g = int(np.clip(point[4], 0, 255))
                b = int(np.clip(point[5], 0, 255))
                rgb = (r << 16) | (g << 8) | b
                f.write(f"{x:.6f} {y:.6f} {z:.6f} {rgb}\n")

        self.get_logger().info(f"PCD file saved: {filepath}")

    def clear_points_callback(self, request, response):
        """Clear accumulated points."""
        self.accumulated_points = []
        self.frame_count = 0
        response.success = True
        response.message = "Points cleared"
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = StereoDepthNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

