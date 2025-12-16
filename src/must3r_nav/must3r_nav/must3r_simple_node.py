#!/usr/bin/env python3
"""
MUSt3R ROS2 Node - DUMMY MODE ONLY
This node provides a testing interface with dummy point clouds.

⚠️  IMPORTANT: For actual MUSt3R reconstruction, use must3r_bridge_node.py
    which communicates with must3r_processor.py via file-based IPC.

This node exists for:
  - Testing ROS2 integration without MUSt3R
  - Verifying camera topics are publishing
  - Debugging point cloud visualization

Assignment 4 Part 2 - MUSt3R 3D Reconstruction
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import numpy as np
import struct
import sys
import os

# ========================================================================
# CRITICAL: Do NOT run this from must3r_env!
# ========================================================================
if 'must3r_env' in sys.prefix or 'must3r_env' in os.environ.get('VIRTUAL_ENV', ''):
    print("\n" + "="*70)
    print("❌ ERROR: Running from must3r_env virtual environment!")
    print("="*70)
    print("\nYou are in:", sys.prefix)
    print("\nThis ROS2 node CANNOT run from must3r_env due to Python version")
    print("incompatibility (ROS2=Python 3.10, must3r_env=Python 3.11)")
    print("\nTo fix:")
    print("  1. Exit must3r_env:   $ deactivate")
    print("  2. Use ROS2 env:      $ source /opt/ros/humble/setup.bash")
    print("  3. Run this node:     $ ros2 run must3r_nav must3r_simple_node")
    print("\nOr better yet, use must3r_bridge_node instead:")
    print("  $ ros2 run must3r_nav must3r_bridge")
    print("="*70 + "\n")
    sys.exit(1)

# CANNOT import MUSt3R here - Python version incompatibility
# ROS2 uses Python 3.10, MUSt3R needs Python 3.11
MUST3R_AVAILABLE = False
print("⚠️  Running in DUMMY MODE - use must3r_bridge_node for real reconstruction")

# Do NOT import open3d from must3r_env - version conflict
# If needed, install in system: pip install open3d
OPEN3D_AVAILABLE = False
o3d = None

# Optional: Try importing system open3d (not from must3r_env)
try:
    # Only import if it's from system Python, not must3r_env
    import importlib.util
    if importlib.util.find_spec("open3d"):
        spec = importlib.util.find_spec("open3d")
        # Check if it's NOT from must3r_env
        if spec and spec.origin and "must3r_env" not in spec.origin:
            import open3d as o3d
            OPEN3D_AVAILABLE = True
            print("✓ Open3D available for PCD export (system)")
        else:
            print("⚠ Open3D found but from must3r_env - skipping to avoid version conflict")
except (ImportError, OSError) as e:
    print(f"⚠ Open3D not available - PCD export will use numpy")


class MUSt3RMultiCameraNode(Node):
    """
    MUSt3R node with support for multiple cameras and PCD export.
    """

    def __init__(self):
        super().__init__('must3r_node')

        # Parameters
        self.declare_parameter('reconstruction_rate', 0.5)  # Hz (slower for CPU)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_width', 512)
        self.declare_parameter('image_height', 512)
        self.declare_parameter('pcd_output_dir', '/home/hariprasad/ros2_ws/pointclouds')
        self.declare_parameter('auto_save_pcd', False)

        self.recon_rate = self.get_parameter('reconstruction_rate').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        self.pcd_output_dir = self.get_parameter('pcd_output_dir').value
        self.auto_save_pcd = self.get_parameter('auto_save_pcd').value

        # Create output directory
        os.makedirs(self.pcd_output_dir, exist_ok=True)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Image buffers for BOTH cameras
        self.base_left_image = None
        self.base_right_image = None
        self.arm_left_image = None
        self.arm_right_image = None
        self.image_count = 0

        # MUSt3R model components
        self.encoder = None
        self.decoder = None
        self.model = None
        self.device = 'cpu'  # Use CPU for compatibility

        # Accumulated point cloud for PCD export
        self.accumulated_points = []
        self.pcd_save_counter = 0

        # ============================================================
        # Subscribers for ZED Base Camera (on Husky)
        # ============================================================
        self.base_left_sub = self.create_subscription(
            Image,
            '/zed_base/left/image_raw',
            self.base_left_callback,
            10
        )
        self.base_right_sub = self.create_subscription(
            Image,
            '/zed_base/right/image_raw',
            self.base_right_callback,
            10
        )

        # ============================================================
        # Subscribers for ZED Arm Camera (on UR3 end effector)
        # ============================================================
        self.arm_left_sub = self.create_subscription(
            Image,
            '/zed_arm/left/image_raw',
            self.arm_left_callback,
            10
        )
        self.arm_right_sub = self.create_subscription(
            Image,
            '/zed_arm/right/image_raw',
            self.arm_right_callback,
            10
        )

        # ============================================================
        # Publishers
        # ============================================================
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/must3r/point_cloud',
            10
        )
        self.base_pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/must3r/base_point_cloud',
            10
        )
        self.arm_pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/must3r/arm_point_cloud',
            10
        )

        # ============================================================
        # Services for PCD export
        # ============================================================
        self.save_pcd_srv = self.create_service(
            Trigger,
            '/must3r/save_pcd',
            self.save_pcd_callback
        )
        self.clear_points_srv = self.create_service(
            Trigger,
            '/must3r/clear_points',
            self.clear_points_callback
        )

        # Timer for reconstruction
        self.timer = self.create_timer(
            1.0 / self.recon_rate,
            self.reconstruction_callback
        )

        # MUSt3R model
        self.model = None
        self.device = 'cpu'  # Force CPU since no CUDA

        self.get_logger().info("=" * 70)
        self.get_logger().info("MUSt3R Multi-Camera Node Started")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Reconstruction rate: {self.recon_rate} Hz")
        self.get_logger().info(f"MUSt3R available: {MUST3R_AVAILABLE}")
        self.get_logger().info(f"Open3D available: {OPEN3D_AVAILABLE}")
        self.get_logger().info(f"Device: {self.device}")
        self.get_logger().info(f"PCD output directory: {self.pcd_output_dir}")
        self.get_logger().info("")
        self.get_logger().info("Subscribed to cameras:")
        self.get_logger().info("  Base camera: /zed_base/left/image_raw, /zed_base/right/image_raw")
        self.get_logger().info("  Arm camera:  /zed_arm/left/image_raw, /zed_arm/right/image_raw")
        self.get_logger().info("")
        self.get_logger().info("Publishing to:")
        self.get_logger().info("  /must3r/point_cloud (combined)")
        self.get_logger().info("  /must3r/base_point_cloud (base camera only)")
        self.get_logger().info("  /must3r/arm_point_cloud (arm camera only)")
        self.get_logger().info("")
        self.get_logger().info("Services:")
        self.get_logger().info("  /must3r/save_pcd - Save accumulated points to PCD file")
        self.get_logger().info("  /must3r/clear_points - Clear accumulated points")
        self.get_logger().info("=" * 70)

    # ================================================================
    # Image Callbacks
    # ================================================================

    def base_left_callback(self, msg: Image):
        try:
            self.base_left_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            self.image_count += 1
            if self.image_count % 100 == 0:
                self.get_logger().info(f"Received {self.image_count} images total")
        except Exception as e:
            self.get_logger().error(f"Base left image error: {e}")

    def base_right_callback(self, msg: Image):
        try:
            self.base_right_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.get_logger().error(f"Base right image error: {e}")

    def arm_left_callback(self, msg: Image):
        try:
            self.arm_left_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.get_logger().error(f"Arm left image error: {e}")

    def arm_right_callback(self, msg: Image):
        try:
            self.arm_right_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.get_logger().error(f"Arm right image error: {e}")

    # ================================================================
    # Reconstruction
    # ================================================================

    def reconstruction_callback(self):
        """Periodic reconstruction from camera images"""

        # Check if we have base camera images
        has_base = self.base_left_image is not None and self.base_right_image is not None
        has_arm = self.arm_left_image is not None and self.arm_right_image is not None

        if not has_base and not has_arm:
            if self.image_count == 0:
                self.get_logger().warn(
                    "Waiting for camera images... Is Gazebo running?"
                )
            return

        all_points = []

        # Reconstruct from base camera
        if has_base:
            base_points = self.run_reconstruction(
                self.base_left_image, 
                self.base_right_image,
                "base"
            )
            if base_points is not None and len(base_points) > 0:
                # Publish base point cloud
                pc_msg = self.create_pointcloud2_msg(base_points, 'zed_base_left_camera_frame')
                self.base_pointcloud_pub.publish(pc_msg)
                all_points.append(base_points)
                self.get_logger().info(f"Base camera: {len(base_points)} points")

        # Reconstruct from arm camera
        if has_arm:
            arm_points = self.run_reconstruction(
                self.arm_left_image, 
                self.arm_right_image,
                "arm"
            )
            if arm_points is not None and len(arm_points) > 0:
                # Publish arm point cloud
                pc_msg = self.create_pointcloud2_msg(arm_points, 'zed_arm_left_camera_frame')
                self.arm_pointcloud_pub.publish(pc_msg)
                all_points.append(arm_points)
                self.get_logger().info(f"Arm camera: {len(arm_points)} points")

        # Combine and publish all points
        if all_points:
            combined = np.vstack(all_points)
            pc_msg = self.create_pointcloud2_msg(combined, 'base_link')
            self.pointcloud_pub.publish(pc_msg)
            self.get_logger().info(f"✓ Published combined point cloud: {len(combined)} points")

            # Accumulate for PCD export
            self.accumulated_points.append(combined)

            # Auto-save if enabled
            if self.auto_save_pcd and len(self.accumulated_points) >= 10:
                self._save_pcd_internal()

    def run_reconstruction(self, left_img, right_img, camera_name):
        """
        Run 3D reconstruction from stereo images.
        Returns numpy array of 3D points or None.

        ⚠️  This node ONLY generates dummy point clouds for testing.
        For real MUSt3R reconstruction, use must3r_bridge_node.py
        """
        return self.create_dummy_pointcloud(left_img)

    def create_dummy_pointcloud(self, img=None):
        """Create dummy point cloud for testing"""
        points = []
        for x in np.linspace(-2, 2, 15):
            for y in np.linspace(-2, 2, 15):
                for z in np.linspace(0, 2, 8):
                    if img is not None:
                        ix = int((x + 2) / 4 * img.shape[1])
                        iy = int((y + 2) / 4 * img.shape[0])
                        ix = np.clip(ix, 0, img.shape[1] - 1)
                        iy = np.clip(iy, 0, img.shape[0] - 1)
                        intensity = np.mean(img[iy, ix]) / 255.0
                        z_mod = z + intensity * 0.5
                    else:
                        z_mod = z
                    points.append([x, y, z_mod])
        return np.array(points, dtype=np.float32)

    # ================================================================
    # PCD Export
    # ================================================================

    def save_pcd_callback(self, request, response):
        """Service callback to save PCD file"""
        success, message = self._save_pcd_internal()
        response.success = success
        response.message = message
        return response

    def clear_points_callback(self, request, response):
        """Service callback to clear accumulated points"""
        self.accumulated_points = []
        response.success = True
        response.message = "Accumulated points cleared"
        self.get_logger().info("Accumulated points cleared")
        return response

    def _save_pcd_internal(self):
        """Internal method to save PCD file"""
        if not OPEN3D_AVAILABLE:
            return False, "Open3D not available"

        if not self.accumulated_points:
            return False, "No points accumulated"

        try:
            # Combine all accumulated points
            all_points = np.vstack(self.accumulated_points)
            
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(all_points)

            # Generate filename
            self.pcd_save_counter += 1
            filename = os.path.join(
                self.pcd_output_dir,
                f"mars_reconstruction_{self.pcd_save_counter:04d}.pcd"
            )

            # Save PCD file
            o3d.io.write_point_cloud(filename, pcd)

            msg = f"Saved {len(all_points)} points to {filename}"
            self.get_logger().info(f"✓ {msg}")

            # Clear accumulated points after save
            self.accumulated_points = []

            return True, msg

        except Exception as e:
            msg = f"PCD save failed: {e}"
            self.get_logger().error(msg)
            return False, msg

    # ================================================================
    # Point Cloud Message Creation
    # ================================================================

    def create_pointcloud2_msg(self, points: np.ndarray, frame_id: str) -> PointCloud2:
        """Convert numpy array to PointCloud2 message"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        point_struct = struct.Struct('fff')
        buffer = bytearray(point_struct.size * len(points))

        for i, point in enumerate(points):
            point_struct.pack_into(buffer, i * point_struct.size, *point)

        pc_msg = PointCloud2()
        pc_msg.header = header
        pc_msg.height = 1
        pc_msg.width = len(points)
        pc_msg.fields = fields
        pc_msg.is_bigendian = False
        pc_msg.point_step = point_struct.size
        pc_msg.row_step = point_struct.size * len(points)
        pc_msg.is_dense = True
        pc_msg.data = bytes(buffer)

        return pc_msg


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MUSt3RMultiCameraNode()
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
