# Assignment 4 Part 2: MUSt3R Implementation Guide
## Step-by-Step Implementation Roadmap

**Course:** CS498GC Mobile Robotics - Fall 2025
**Due Date:** December 9, 2025 @ 11:59 PM
**Focus:** 3D Reconstruction using MUSt3R (NOT 2D SLAM)

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Phase 1: Environment Setup](#phase-1-environment-setup)
3. [Phase 2: ZED Camera Integration](#phase-2-zed-camera-integration)
4. [Phase 3: MUSt3R Integration](#phase-3-must3r-integration)
5. [Phase 4: Point Cloud Processing](#phase-4-point-cloud-processing)
6. [Phase 5: Vision-based Navigation](#phase-5-vision-based-navigation)
7. [Phase 6: Testing and Documentation](#phase-6-testing-and-documentation)
8. [Troubleshooting](#troubleshooting)

---

## Quick Start

### What's Different from Part 1?

**Part 1:** Mobile manipulator with basic control
**Part 2:** Add 3D vision system with MUSt3R reconstruction

### What You'll Build

```
ZED Camera → Multi-view Images → MUSt3R Model → 3D Point Clouds
                                      ↓
                            Obstacle Detection
                                      ↓
                            Path Planning & Navigation
```

### Time Estimate

- **Phase 1** (Environment): 2-3 hours
- **Phase 2** (ZED Camera): 3-4 hours
- **Phase 3** (MUSt3R): 4-6 hours
- **Phase 4** (Point Clouds): 4-5 hours
- **Phase 5** (Navigation): 6-8 hours
- **Phase 6** (Testing/Docs): 4-6 hours
- **Total**: 23-32 hours

Start early!

---

## Phase 1: Environment Setup

### Step 1.1: Check System Requirements

```bash
# Check Ubuntu version
lsb_release -a
# Should be: Ubuntu 22.04 (for ROS2 Humble) or 24.04 (for Jazzy)

# Check Python version
python3 --version
# Should be: Python 3.11.x

# Check CUDA availability (optional but recommended)
nvidia-smi
# Should show GPU info if available

# Check ROS2 installation
ros2 --version
# Should show: ros2 run 2.x.x (Humble or Jazzy)
```

### Step 1.2: Install Python 3.11 (if needed)

```bash
# Install Python 3.11
sudo apt update
sudo apt install python3.11 python3.11-venv python3.11-dev

# Verify installation
python3.11 --version
```

### Step 1.3: Create Python Virtual Environment

```bash
# Create virtual environment for MUSt3R
python3.11 -m venv ~/must3r_env

# Activate environment
source ~/must3r_env/bin/activate

# Upgrade pip
pip install --upgrade pip setuptools wheel
```

### Step 1.4: Install PyTorch 2.7.0

```bash
# Activate virtual environment
source ~/must3r_env/bin/activate

# Install PyTorch with CUDA 12.1 support (if GPU available)
pip install torch==2.7.0 torchvision==0.18.0 --index-url https://download.pytorch.org/whl/cu121

# OR install CPU-only version (slower)
# pip install torch==2.7.0 torchvision==0.18.0 --index-url https://download.pytorch.org/whl/cpu

# Verify installation
python -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"
```

**Expected output:**
```
PyTorch: 2.7.0+cu121
CUDA: True
```

### Step 1.5: Install MUSt3R

```bash
# Install MUSt3R from GitHub
pip install must3r@git+https://github.com/naver/must3r.git

# Install additional dependencies
pip install opencv-python numpy scipy open3d matplotlib pillow

# Verify installation
python -c "from must3r import MUST3R; print('MUSt3R installed successfully!')"
```

### Step 1.6: Install ROS2 Python Packages

```bash
# Still in virtual environment
pip install rospkg transforms3d

# Install cv_bridge dependencies
pip install pyyaml empy lark
```

**Checkpoint 1:** Environment is ready with Python 3.11, PyTorch 2.7.0, and MUSt3R installed.

---

## Phase 2: ZED Camera Integration

### Step 2.1: Install ZED SDK 5.1

Visit: https://www.stereolabs.com/developers/release

```bash
# Download ZED SDK for Ubuntu 22.04
cd ~/Downloads
wget https://download.stereolabs.com/zedsdk/5.1/cu121/ubuntu22

# Make executable
chmod +x ubuntu22

# Run installer
./ubuntu22

# Follow prompts:
# - Install ZED SDK: Yes
# - Install CUDA (if not installed): Yes
# - Install Python API: Yes
# - Install samples: Optional
```

### Step 2.2: Verify ZED SDK Installation

```bash
# Run ZED diagnostic tool
/usr/local/zed/tools/ZED_Diagnostic

# Expected: Shows ZED SDK version 5.1.x
```

### Step 2.3: Clone ZED ROS2 Wrapper

```bash
# Navigate to workspace
cd ~/ros2_ws/src/

# Clone ZED ROS2 wrapper
git clone https://github.com/stereolabs/zed-ros2-wrapper.git

# Check branch (should be humble or jazzy)
cd zed-ros2-wrapper
git branch -a
```

### Step 2.4: Install Dependencies

```bash
cd ~/ros2_ws

# Update package index
sudo apt update

# Update rosdep
rosdep update

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Step 2.5: Build ZED Wrapper

```bash
cd ~/ros2_ws

# Build with optimizations
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

# Source the installation
source install/setup.bash

# Add to bashrc
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
```

### Step 2.6: Test ZED Camera (if physical camera available)

```bash
# Launch ZED node
ros2 launch zed_wrapper zed_camera.launch.py

# In another terminal, check topics
ros2 topic list | grep zed

# Expected topics:
# /zed/left/image_rect_color
# /zed/right/image_rect_color
# /zed/depth/depth_registered
# /zed/point_cloud/cloud_registered
# /zed/zed_node/left/camera_info
# /zed/zed_node/right/camera_info
```

### Step 2.7: View ZED Images in RViz2

```bash
# Launch RViz2
rviz2

# Add Image display:
# - Click "Add" button
# - Select "Image"
# - Set Topic: /zed/left/image_rect_color

# You should see live camera feed
```

**Checkpoint 2:** ZED camera is installed and publishing images to ROS2.

---

## Phase 3: MUSt3R Integration

### Step 3.1: Create ROS2 Package for MUSt3R

```bash
cd ~/ros2_ws/src

# Create Python package
ros2 pkg create must3r_nav \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs geometry_msgs std_msgs visualization_msgs

# Create package structure
cd must3r_nav
mkdir -p launch config
```

### Step 3.2: Create MUSt3R Processor Class

Create file: `must3r_nav/must3r_processor.py`

```python
"""
MUSt3R Processor for 3D Reconstruction
Wraps MUSt3R model for ROS2 integration
"""

import torch
import numpy as np
from must3r import MUST3R
from typing import List, Tuple
import cv2


class MUST3RProcessor:
    """Handles MUSt3R model initialization and inference"""

    def __init__(self, device: str = 'auto'):
        """
        Initialize MUSt3R processor

        Args:
            device: 'cuda', 'cpu', or 'auto' (auto-detect)
        """
        # Determine device
        if device == 'auto':
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = device

        print(f"[MUSt3R] Initializing on device: {self.device}")

        # Load pre-trained model
        self.model = MUST3R.from_pretrained(
            "naver/MUSt3R_ViTLarge_BaseDecoder_512_dpt"
        ).to(self.device)

        self.model.eval()  # Set to evaluation mode

        # Image preprocessing parameters
        self.target_size = (512, 512)  # MUSt3R expects 512x512

        print("[MUSt3R] Model loaded successfully")

    def preprocess_images(self, images: List[np.ndarray]) -> torch.Tensor:
        """
        Preprocess images for MUSt3R input

        Args:
            images: List of RGB images (H, W, 3) as numpy arrays

        Returns:
            Preprocessed tensor (B, 3, H, W)
        """
        processed = []

        for img in images:
            # Resize to 512x512
            img_resized = cv2.resize(img, self.target_size)

            # Convert to float and normalize to [0, 1]
            img_normalized = img_resized.astype(np.float32) / 255.0

            # Convert to torch tensor (H, W, C) -> (C, H, W)
            img_tensor = torch.from_numpy(img_normalized).permute(2, 0, 1)

            processed.append(img_tensor)

        # Stack into batch (B, C, H, W)
        batch = torch.stack(processed).to(self.device)

        return batch

    def reconstruct_scene(
        self, images: List[np.ndarray]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Perform 3D reconstruction from multi-view images

        Args:
            images: List of RGB images (numpy arrays)

        Returns:
            Tuple of:
            - pointmap: 3D coordinates (N, 3) as numpy array
            - confidences: Confidence values (N,) as numpy array
        """
        if len(images) < 2:
            raise ValueError("MUSt3R requires at least 2 views")

        # Preprocess images
        image_batch = self.preprocess_images(images)

        # Run inference
        with torch.no_grad():
            pointmaps, confidences = self.model(image_batch)

        # Convert to numpy
        pointmap_np = pointmaps.cpu().numpy()
        confidence_np = confidences.cpu().numpy()

        # Reshape pointmap from (B, H, W, 3) to (N, 3)
        B, H, W, _ = pointmap_np.shape
        pointmap_flat = pointmap_np.reshape(-1, 3)
        confidence_flat = confidence_np.reshape(-1)

        return pointmap_flat, confidence_flat

    def filter_by_confidence(
        self,
        pointmap: np.ndarray,
        confidences: np.ndarray,
        threshold: float = 0.5
    ) -> np.ndarray:
        """
        Filter point cloud by confidence threshold

        Args:
            pointmap: 3D points (N, 3)
            confidences: Confidence values (N,)
            threshold: Minimum confidence (0-1)

        Returns:
            Filtered pointmap (M, 3) where M <= N
        """
        mask = confidences > threshold
        return pointmap[mask]


if __name__ == "__main__":
    # Test MUSt3R processor
    processor = MUST3RProcessor()
    print("MUSt3R processor initialized successfully!")
```

### Step 3.3: Create MUSt3R ROS2 Node

Create file: `must3r_nav/must3r_node.py`

```python
"""
MUSt3R ROS2 Node
Subscribes to camera images, performs 3D reconstruction, publishes point clouds
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
from .must3r_processor import MUST3RProcessor
import struct


class MUST3RNode(Node):
    """ROS2 node for MUSt3R 3D reconstruction"""

    def __init__(self):
        super().__init__('must3r_node')

        # Parameters
        self.declare_parameter('device', 'auto')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('reconstruction_rate', 1.0)  # Hz

        device = self.get_parameter('device').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        recon_rate = self.get_parameter('reconstruction_rate').value

        # Initialize MUSt3R processor
        self.get_logger().info(f"Initializing MUSt3R on device: {device}")
        self.processor = MUST3RProcessor(device=device)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Image buffer for multi-view reconstruction
        self.image_buffer = []
        self.max_buffer_size = 2  # Minimum 2 views

        # Subscribers
        self.image_sub_left = self.create_subscription(
            Image,
            '/zed/left/image_rect_color',
            self.left_image_callback,
            10
        )

        self.image_sub_right = self.create_subscription(
            Image,
            '/zed/right/image_rect_color',
            self.right_image_callback,
            10
        )

        # Publishers
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/must3r/point_cloud',
            10
        )

        # Timer for periodic reconstruction
        reconstruction_period = 1.0 / recon_rate
        self.timer = self.create_timer(reconstruction_period, self.reconstruction_callback)

        self.get_logger().info("MUSt3R node initialized")

    def left_image_callback(self, msg: Image):
        """Callback for left camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            # Store as first view
            if len(self.image_buffer) < 1:
                self.image_buffer.append(cv_image)
            else:
                self.image_buffer[0] = cv_image
        except Exception as e:
            self.get_logger().error(f"Error converting left image: {e}")

    def right_image_callback(self, msg: Image):
        """Callback for right camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            # Store as second view
            if len(self.image_buffer) < 2:
                self.image_buffer.append(cv_image)
            else:
                self.image_buffer[1] = cv_image
        except Exception as e:
            self.get_logger().error(f"Error converting right image: {e}")

    def reconstruction_callback(self):
        """Periodic reconstruction from buffered images"""
        if len(self.image_buffer) < self.max_buffer_size:
            self.get_logger().warn(
                f"Insufficient images for reconstruction: {len(self.image_buffer)}/{self.max_buffer_size}"
            )
            return

        try:
            # Perform 3D reconstruction
            self.get_logger().info("Running MUSt3R reconstruction...")
            pointmap, confidences = self.processor.reconstruct_scene(self.image_buffer)

            # Filter by confidence
            filtered_pointmap = self.processor.filter_by_confidence(
                pointmap, confidences, self.confidence_threshold
            )

            self.get_logger().info(
                f"Reconstruction complete: {len(filtered_pointmap)} points"
            )

            # Publish point cloud
            pc_msg = self.create_pointcloud2_msg(filtered_pointmap)
            self.pointcloud_pub.publish(pc_msg)

        except Exception as e:
            self.get_logger().error(f"Reconstruction error: {e}")

    def create_pointcloud2_msg(self, points: np.ndarray) -> PointCloud2:
        """
        Convert numpy array to PointCloud2 message

        Args:
            points: Nx3 array of XYZ coordinates

        Returns:
            PointCloud2 message
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'  # Adjust frame as needed

        # Define point cloud fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Pack points into binary data
        point_struct = struct.Struct('fff')
        buffer = bytearray(point_struct.size * len(points))

        for i, point in enumerate(points):
            point_struct.pack_into(buffer, i * point_struct.size, *point)

        # Create PointCloud2 message
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
    node = MUST3RNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3.4: Update Package Configuration

Edit `setup.py`:

```python
from setuptools import setup

package_name = 'must3r_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/must3r_launch.py']),
        ('share/' + package_name + '/config', ['config/must3r_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='MUSt3R 3D Reconstruction for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'must3r_node = must3r_nav.must3r_node:main',
        ],
    },
)
```

### Step 3.5: Create Launch File

Create file: `launch/must3r_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('must3r_nav')
    params_file = os.path.join(pkg_dir, 'config', 'must3r_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='auto',
            description='Device for MUSt3R: cuda, cpu, or auto'
        ),

        Node(
            package='must3r_nav',
            executable='must3r_node',
            name='must3r_node',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'confidence_threshold': 0.5,
                'reconstruction_rate': 1.0,
            }]
        ),
    ])
```

### Step 3.6: Create Configuration File

Create file: `config/must3r_params.yaml`

```yaml
must3r_node:
  ros__parameters:
    device: 'auto'  # 'cuda', 'cpu', or 'auto'
    confidence_threshold: 0.5  # 0-1 range
    reconstruction_rate: 1.0  # Hz
    max_buffer_size: 2  # Number of views
```

### Step 3.7: Build and Test MUSt3R Node

```bash
# Build package
cd ~/ros2_ws
colcon build --packages-select must3r_nav
source install/setup.bash

# Launch MUSt3R node (requires ZED camera running)
# Terminal 1: ZED camera
ros2 launch zed_wrapper zed_camera.launch.py

# Terminal 2: MUSt3R node
ros2 launch must3r_nav must3r_launch.py

# Terminal 3: Check point cloud topic
ros2 topic list | grep must3r
ros2 topic echo /must3r/point_cloud --once
```

**Checkpoint 3:** MUSt3R is integrated with ROS2 and generating point clouds.

---

## Phase 4: Point Cloud Processing

### Step 4.1: Install Point Cloud Library (PCL) for ROS2

```bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
```

### Step 4.2: Create Point Cloud Processor

Create file: `must3r_nav/point_cloud_processor.py`

```python
"""
Point Cloud Processing
Filters, segments, and processes MUSt3R point clouds for navigation
"""

import numpy as np
from scipy.spatial import KDTree
import open3d as o3d


class PointCloudProcessor:
    """Processes point clouds for obstacle detection and navigation"""

    def __init__(self):
        self.voxel_size = 0.05  # 5cm voxel size
        self.outlier_neighbors = 20
        self.outlier_std_ratio = 2.0
        self.ground_threshold = 0.1  # meters

    def voxel_filter(self, points: np.ndarray) -> np.ndarray:
        """
        Downsample point cloud using voxel grid filter

        Args:
            points: Nx3 point cloud

        Returns:
            Downsampled point cloud
        """
        if len(points) == 0:
            return points

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Apply voxel downsampling
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        return np.asarray(pcd_down.points)

    def remove_outliers(self, points: np.ndarray) -> np.ndarray:
        """
        Remove statistical outliers from point cloud

        Args:
            points: Nx3 point cloud

        Returns:
            Filtered point cloud
        """
        if len(points) < self.outlier_neighbors:
            return points

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Remove outliers
        pcd_filtered, _ = pcd.remove_statistical_outlier(
            nb_neighbors=self.outlier_neighbors,
            std_ratio=self.outlier_std_ratio
        )

        return np.asarray(pcd_filtered.points)

    def segment_ground(self, points: np.ndarray) -> tuple:
        """
        Segment ground plane from obstacles

        Args:
            points: Nx3 point cloud

        Returns:
            Tuple of (ground_points, obstacle_points)
        """
        if len(points) == 0:
            return np.array([]), np.array([])

        # Simple height-based segmentation
        # Assume ground is near z=0 (adjust based on your coordinate frame)
        z_coords = points[:, 2]

        ground_mask = np.abs(z_coords) < self.ground_threshold
        obstacle_mask = ~ground_mask

        ground_points = points[ground_mask]
        obstacle_points = points[obstacle_mask]

        return ground_points, obstacle_points

    def process_pipeline(self, points: np.ndarray) -> dict:
        """
        Full processing pipeline

        Args:
            points: Raw Nx3 point cloud from MUSt3R

        Returns:
            Dictionary with processed point clouds:
            - 'filtered': Voxelized and outlier-removed
            - 'ground': Ground plane points
            - 'obstacles': Obstacle points
        """
        # Step 1: Voxel filtering
        filtered = self.voxel_filter(points)

        # Step 2: Remove outliers
        filtered = self.remove_outliers(filtered)

        # Step 3: Segment ground and obstacles
        ground, obstacles = self.segment_ground(filtered)

        return {
            'filtered': filtered,
            'ground': ground,
            'obstacles': obstacles,
            'num_points': {
                'raw': len(points),
                'filtered': len(filtered),
                'ground': len(ground),
                'obstacles': len(obstacles)
            }
        }


if __name__ == "__main__":
    # Test point cloud processor
    processor = PointCloudProcessor()

    # Generate test point cloud
    test_points = np.random.rand(10000, 3) * 10  # 10000 random points
    result = processor.process_pipeline(test_points)

    print(f"Point cloud processing test:")
    print(f"  Raw points: {result['num_points']['raw']}")
    print(f"  Filtered points: {result['num_points']['filtered']}")
    print(f"  Ground points: {result['num_points']['ground']}")
    print(f"  Obstacle points: {result['num_points']['obstacles']}")
```

### Step 4.3: Add Point Cloud Visualization

Create file: `must3r_nav/point_cloud_visualizer.py`

```python
"""
Point Cloud Visualization Node
Publishes processed point clouds with colors for RViz2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
from .point_cloud_processor import PointCloudProcessor


class PointCloudVisualizer(Node):
    """Visualizes processed point clouds in RViz2"""

    def __init__(self):
        super().__init__('pointcloud_visualizer')

        # Processor
        self.processor = PointCloudProcessor()

        # Subscribers
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/must3r/point_cloud',
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.filtered_pub = self.create_publisher(
            PointCloud2, '/must3r/filtered', 10
        )
        self.ground_pub = self.create_publisher(
            PointCloud2, '/must3r/ground', 10
        )
        self.obstacles_pub = self.create_publisher(
            PointCloud2, '/must3r/obstacles', 10
        )

        self.get_logger().info("Point cloud visualizer initialized")

    def pointcloud2_to_array(self, pc_msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 message to numpy array"""
        # Simplified conversion (assumes XYZ fields)
        point_step = pc_msg.point_step
        num_points = pc_msg.width

        points = np.zeros((num_points, 3))

        for i in range(num_points):
            offset = i * point_step
            x = struct.unpack_from('f', pc_msg.data, offset)[0]
            y = struct.unpack_from('f', pc_msg.data, offset + 4)[0]
            z = struct.unpack_from('f', pc_msg.data, offset + 8)[0]
            points[i] = [x, y, z]

        return points

    def array_to_pointcloud2(
        self, points: np.ndarray, color: tuple = None
    ) -> PointCloud2:
        """Convert numpy array to PointCloud2 with optional color"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        if color is not None:
            # XYZRGB format
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]

            # Pack RGB into uint32
            r, g, b = color
            rgb = (int(r) << 16) | (int(g) << 8) | int(b)

            point_struct = struct.Struct('fffI')
            buffer = bytearray(point_struct.size * len(points))

            for i, point in enumerate(points):
                point_struct.pack_into(buffer, i * point_struct.size, *point, rgb)
        else:
            # XYZ only
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

    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud and publish visualizations"""
        try:
            # Convert to numpy array
            points = self.pointcloud2_to_array(msg)

            # Process point cloud
            result = self.processor.process_pipeline(points)

            # Publish filtered (white)
            if len(result['filtered']) > 0:
                filtered_msg = self.array_to_pointcloud2(result['filtered'])
                self.filtered_pub.publish(filtered_msg)

            # Publish ground (green)
            if len(result['ground']) > 0:
                ground_msg = self.array_to_pointcloud2(
                    result['ground'], color=(0, 255, 0)
                )
                self.ground_pub.publish(ground_msg)

            # Publish obstacles (red)
            if len(result['obstacles']) > 0:
                obstacles_msg = self.array_to_pointcloud2(
                    result['obstacles'], color=(255, 0, 0)
                )
                self.obstacles_pub.publish(obstacles_msg)

            self.get_logger().info(
                f"Processed: {result['num_points']['obstacles']} obstacles, "
                f"{result['num_points']['ground']} ground"
            )

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4.4: Update setup.py with New Nodes

Add to `entry_points` in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'must3r_node = must3r_nav.must3r_node:main',
        'pointcloud_visualizer = must3r_nav.point_cloud_visualizer:main',
    ],
},
```

### Step 4.5: Build and Test Processing

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select must3r_nav
source install/setup.bash

# Launch full pipeline
# Terminal 1: ZED camera
ros2 launch zed_wrapper zed_camera.launch.py

# Terminal 2: MUSt3R reconstruction
ros2 launch must3r_nav must3r_launch.py

# Terminal 3: Point cloud processing
ros2 run must3r_nav pointcloud_visualizer

# Terminal 4: Visualize in RViz2
rviz2
# Add PointCloud2 displays:
# - /must3r/ground (green)
# - /must3r/obstacles (red)
```

**Checkpoint 4:** Point clouds are processed, filtered, and visualized with ground/obstacle segmentation.

---

## Phase 5: Vision-based Navigation

(This section would continue with obstacle detection and path planning implementation...)

**Due to length, I'll provide the key structure:**

### Step 5.1: Obstacle Detector Node
- Detect obstacles from segmented point clouds
- Compute safe zones
- Publish obstacle markers

### Step 5.2: Path Planner
- Use processed obstacles for navigation
- Implement simple potential field or A* in 3D
- Generate velocity commands

### Step 5.3: Integration with Existing Navigation
- Interface with Nav2 (if needed)
- Or standalone navigation controller

---

## Phase 6: Testing and Documentation

### Step 6.1: Record ROSbag
```bash
ros2 bag record -o must3r_demo \
  /zed/left/image_rect_color \
  /zed/right/image_rect_color \
  /must3r/point_cloud \
  /must3r/obstacles \
  /cmd_vel \
  /tf
```

### Step 6.2: Create Demo Video
- Screen recording showing:
  - Camera views
  - 3D reconstruction in RViz2
  - Robot navigation
  - Obstacle avoidance

### Step 6.3: Write Technical Report
- Use IEEE template
- 4 pages
- Include metrics and results

---

## Troubleshooting

### PyTorch/CUDA Issues
```bash
# Verify CUDA
python -c "import torch; print(torch.cuda.is_available())"

# Reinstall if needed
pip uninstall torch torchvision
pip install torch==2.7.0 torchvision==0.18.0 --index-url https://download.pytorch.org/whl/cu121
```

### ZED Camera Issues
```bash
# Run diagnostic
/usr/local/zed/tools/ZED_Diagnostic

# Check permissions
sudo chmod 666 /dev/video*
```

### Point Cloud Not Showing in RViz2
- Check frame_id matches (base_link, zed_left_camera_frame, etc.)
- Verify TF tree: `ros2 run tf2_tools view_frames`
- Check topic data: `ros2 topic echo /must3r/point_cloud --once`

---

**Next Steps:** Continue with Phase 5 (Navigation) implementation, then proceed to testing and documentation.

*Last updated: December 4, 2025*
