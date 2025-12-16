#!/usr/bin/env python3
"""
MUSt3R Bridge Node - ROS2 side (Python 3.10)
Captures camera images, saves to shared directory,
reads point clouds from MUSt3R processor.

This bridges the Python 3.11 (MUSt3R) and Python 3.10 (ROS2) gap.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import numpy as np
import struct
import os
import json
from pathlib import Path
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


# Shared directories (same as must3r_processor.py)
SHARED_DIR = Path("/tmp/must3r_bridge")
IMAGE_DIR = SHARED_DIR / "images"
POINTS_DIR = SHARED_DIR / "points"
STATUS_FILE = SHARED_DIR / "status.json"
PCD_DIR = Path.home() / "ros2_ws" / "pointclouds"


class MUSt3RBridgeNode(Node):
    """
    ROS2 node that bridges camera images to MUSt3R processor.
    """

    def __init__(self):
        super().__init__('must3r_bridge_node')

        self.get_logger().info("=" * 60)
        self.get_logger().info("MUSt3R Bridge Node Started")
        self.get_logger().info("=" * 60)

        # Create directories
        IMAGE_DIR.mkdir(parents=True, exist_ok=True)
        POINTS_DIR.mkdir(parents=True, exist_ok=True)
        PCD_DIR.mkdir(parents=True, exist_ok=True)

        # CV Bridge
        self.bridge = CvBridge()

        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Image storage
        self.base_left = None
        self.base_right = None
        self.arm_left = None
        self.arm_right = None

        # Point cloud accumulation
        self.accumulated_points = []

        # Image counter for filenames
        self.image_counter = 0

        # Subscribers for base camera
        self.base_left_sub = self.create_subscription(
            Image, '/zed_base/left/image_raw', self.base_left_cb, 10)
        self.base_right_sub = self.create_subscription(
            Image, '/zed_base/right/image_raw', self.base_right_cb, 10)

        # Subscribers for arm camera
        self.arm_left_sub = self.create_subscription(
            Image, '/zed_arm/left/image_raw', self.arm_left_cb, 10)
        self.arm_right_sub = self.create_subscription(
            Image, '/zed_arm/right/image_raw', self.arm_right_cb, 10)

        # Publishers
        self.combined_pc_pub = self.create_publisher(PointCloud2, '/must3r/point_cloud', 10)
        self.base_pc_pub = self.create_publisher(PointCloud2, '/must3r/base_point_cloud', 10)
        self.arm_pc_pub = self.create_publisher(PointCloud2, '/must3r/arm_point_cloud', 10)

        # Services for PCD export
        self.save_pcd_service = self.create_service(
            Trigger, '/must3r/save_pcd', self.save_pcd_callback)
        self.clear_points_service = self.create_service(
            Trigger, '/must3r/clear_points', self.clear_points_callback)

        # Timer to save images to processor (2 Hz)
        self.save_timer = self.create_timer(0.5, self.save_images_callback)

        # Timer to check for processed point clouds (10 Hz)
        self.read_timer = self.create_timer(0.1, self.read_points_callback)

        # Status timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.check_processor_status)

        # Cleanup timer - remove old files every 30 seconds
        self.cleanup_timer = self.create_timer(30.0, self.cleanup_old_files)

        self.get_logger().info(f"Saving images to: {IMAGE_DIR}")
        self.get_logger().info(f"Reading points from: {POINTS_DIR}")
        self.get_logger().info(f"PCD export dir: {PCD_DIR}")
        self.get_logger().info("=" * 60)

    def base_left_cb(self, msg):
        self.base_left = msg

    def base_right_cb(self, msg):
        self.base_right = msg

    def arm_left_cb(self, msg):
        self.arm_left = msg

    def arm_right_cb(self, msg):
        self.arm_right = msg

    def save_images_callback(self):
        """Save camera images for MUSt3R processor"""
        try:
            # Save base camera images
            if self.base_left is not None and self.base_right is not None:
                self.save_image_pair(self.base_left, self.base_right, "base")
                self.base_left = None
                self.base_right = None

            # Save arm camera images
            if self.arm_left is not None and self.arm_right is not None:
                self.save_image_pair(self.arm_left, self.arm_right, "arm")
                self.arm_left = None
                self.arm_right = None

        except Exception as e:
            self.get_logger().error(f"Error saving images: {e}")

    def save_image_pair(self, left_msg, right_msg, camera_name):
        """Save a stereo image pair to disk"""
        try:
            import cv2

            # Convert ROS messages to OpenCV
            left_cv = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
            right_cv = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")

            # Generate filename
            timestamp = f"{camera_name}_{self.image_counter:06d}"
            self.image_counter += 1

            # Save images
            left_path = IMAGE_DIR / f"{timestamp}_left.png"
            right_path = IMAGE_DIR / f"{timestamp}_right.png"

            cv2.imwrite(str(left_path), left_cv)
            cv2.imwrite(str(right_path), right_cv)

            self.get_logger().debug(f"Saved: {timestamp}")

        except Exception as e:
            self.get_logger().error(f"Error saving image pair: {e}")

    def read_points_callback(self):
        """Read processed point clouds from MUSt3R"""
        try:
            # Find all point files
            point_files = sorted(POINTS_DIR.glob("*.npy"))

            for point_file in point_files:
                try:
                    # Load points
                    points = np.load(point_file)

                    if len(points) == 0:
                        point_file.unlink()
                        continue

                    # Determine camera source from filename
                    if "base_" in point_file.name:
                        camera_frame = "zed_base_left_camera_optical_frame"
                        # Publish in camera frame
                        self.base_pc_pub.publish(self.create_pc2_msg(points, camera_frame))
                        # Transform to base_link for combined
                        transformed_points = self.transform_points(points, camera_frame, "base_link")
                        if transformed_points is not None and len(transformed_points) > 0:
                            self.combined_pc_pub.publish(self.create_pc2_msg(transformed_points, "base_link"))
                            self.accumulated_points.append(transformed_points)
                        else:
                            # Fallback: publish in camera frame if transform fails
                            self.combined_pc_pub.publish(self.create_pc2_msg(points, camera_frame))

                    elif "arm_" in point_file.name:
                        camera_frame = "zed_arm_left_camera_optical_frame"
                        # Publish in camera frame
                        self.arm_pc_pub.publish(self.create_pc2_msg(points, camera_frame))
                        # Transform to base_link for combined
                        transformed_points = self.transform_points(points, camera_frame, "base_link")
                        if transformed_points is not None and len(transformed_points) > 0:
                            self.combined_pc_pub.publish(self.create_pc2_msg(transformed_points, "base_link"))
                            self.accumulated_points.append(transformed_points)
                        else:
                            # Fallback: publish in camera frame if transform fails
                            self.combined_pc_pub.publish(self.create_pc2_msg(points, camera_frame))

                    self.get_logger().info(f"Published {len(points)} points from {point_file.name}")

                    # Remove processed file
                    point_file.unlink()

                except Exception as e:
                    self.get_logger().error(f"Error reading {point_file}: {e}")
                    point_file.unlink()

        except Exception as e:
            self.get_logger().error(f"Error in read_points: {e}")

    def transform_points(self, points, source_frame, target_frame):
        """Transform point cloud from source frame to target frame using TF2"""
        try:
            # Look up the transform from source to target
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )

            # Extract translation and rotation
            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Convert quaternion to rotation matrix
            from scipy.spatial.transform import Rotation as R
            rotation = R.from_quat([rot.x, rot.y, rot.z, rot.w])
            rot_matrix = rotation.as_matrix()

            # Transform XYZ coordinates (points[:, 0:3])
            xyz = points[:, 0:3]
            xyz_transformed = (rot_matrix @ xyz.T).T + np.array([trans.x, trans.y, trans.z])

            # Keep RGB colors unchanged (points[:, 3:6])
            if points.shape[1] >= 6:
                transformed_points = np.hstack([xyz_transformed, points[:, 3:6]])
            else:
                transformed_points = xyz_transformed

            return transformed_points

        except Exception as e:
            self.get_logger().warn(f"TF2 transform failed ({source_frame} -> {target_frame}): {e}")
            return None

    def create_pc2_msg(self, points, frame_id):
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
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])

            if len(point) >= 6:
                r = int(np.clip(point[3], 0, 255))
                g = int(np.clip(point[4], 0, 255))
                b = int(np.clip(point[5], 0, 255))
            else:
                r = g = b = 128

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

    def check_processor_status(self):
        """Check if MUSt3R processor is running"""
        try:
            if STATUS_FILE.exists():
                with open(STATUS_FILE, 'r') as f:
                    status = json.load(f)

                age = os.path.getmtime(STATUS_FILE)
                import time
                if time.time() - age < 5:
                    if status.get("model_loaded"):
                        self.get_logger().debug("MUSt3R processor: running (model loaded)")
                    else:
                        self.get_logger().warn("MUSt3R processor: running (no model)")
                    return

            self.get_logger().warn(
                "MUSt3R processor not running! Start it with:\n"
                "  /home/hariprasad/must3r_env/bin/python "
                "/home/hariprasad/ros2_ws/src/must3r_nav/scripts/must3r_processor.py"
            )
        except Exception as e:
            self.get_logger().debug(f"Status check error: {e}")

    def save_pcd_callback(self, request, response):
        """Save accumulated point clouds to PCD file"""
        try:
            if not self.accumulated_points:
                response.success = False
                response.message = "No points accumulated"
                return response

            # Combine all points
            all_points = np.vstack(self.accumulated_points)

            # Save as numpy (simpler than PCD)
            import time
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            output_path = PCD_DIR / f"pointcloud_{timestamp}.npy"
            np.save(output_path, all_points)

            # Also save as simple PCD format
            pcd_path = PCD_DIR / f"pointcloud_{timestamp}.pcd"
            self.save_simple_pcd(all_points, pcd_path)

            response.success = True
            response.message = f"Saved {len(all_points)} points to {pcd_path}"
            self.get_logger().info(response.message)

            return response

        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            self.get_logger().error(response.message)
            return response

    def save_simple_pcd(self, points, filepath):
        """Save points to simple ASCII PCD format"""
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
                if len(point) >= 6:
                    r, g, b = int(point[3]), int(point[4]), int(point[5])
                else:
                    r = g = b = 128
                rgb = (r << 16) | (g << 8) | b
                f.write(f"{x} {y} {z} {rgb}\n")

    def clear_points_callback(self, request, response):
        """Clear accumulated points"""
        self.accumulated_points = []
        response.success = True
        response.message = "Accumulated points cleared"
        self.get_logger().info(response.message)
        return response

    def cleanup_old_files(self):
        """Remove old image and point files to prevent disk space issues"""
        try:
            import time
            current_time = time.time()
            max_age_seconds = 60  # Remove files older than 60 seconds

            # Cleanup old images
            for img_file in IMAGE_DIR.glob("*.png"):
                if current_time - img_file.stat().st_mtime > max_age_seconds:
                    img_file.unlink()
                    self.get_logger().debug(f"Cleaned up old image: {img_file.name}")

            # Cleanup old points (should already be processed, but just in case)
            for point_file in POINTS_DIR.glob("*.npy"):
                if current_time - point_file.stat().st_mtime > max_age_seconds:
                    point_file.unlink()
                    self.get_logger().debug(f"Cleaned up old points: {point_file.name}")

        except Exception as e:
            self.get_logger().error(f"Cleanup error: {e}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MUSt3RBridgeNode()
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

















