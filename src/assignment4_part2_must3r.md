# Assignment 4 Part 2: MUSt3R 3D Reconstruction

## Overview

This assignment implements **MUSt3R (Multi-view Stereo 3D Reconstruction)** for Mars exploration using a Husky mobile robot equipped with vision sensors. Students will demonstrate 3D scene reconstruction, point cloud processing, and autonomous navigation capabilities.

**Key Change from Previous Version:** This assignment focuses on **3D reconstruction using MUSt3R**, NOT 2D SLAM. The input is RGB images from cameras, and the output is dense 3D point clouds.

## Due Date

**December 9, 2025 @ 11:59 PM CST** (75 points)

## Learning Objectives

- Implement MUSt3R multi-view stereo 3D reconstruction
- Process and filter 3D point clouds for navigation
- Integrate vision sensors (ZED camera) with ROS2
- Develop vision-based obstacle detection and navigation
- Document technical implementation following research standards

## Key Requirements

### System Requirements

- **Python 3.11** environment with **PyTorch 2.7.0**
- **CUDA-capable GPU** (recommended) or CPU mode
- **ROS2 Humble** (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
- **MUSt3R installation**:
  ```bash
  pip install must3r@git+https://github.com/naver/must3r.git
  ```

### Hardware Requirements

- Husky mobile base (existing)
- UR3 robotic arm (existing)
- **ZED Camera** for multi-view image capture (NEW)

---

## Grading Breakdown (75 Points)

### 1. MUSt3R 3D Reconstruction (30 points)

#### Model Setup & Configuration (10 points)

**Excellent (9-10 points):**
- Successfully loads MUSt3R pre-trained model
- Configures for 512x512 resolution
- Implements proper GPU/CPU fallback
- Model loads efficiently with optimization

**Good (7-8 points):**
- Model loads with basic configuration
- Some optimization missing
- Works but not optimal

**Needs Improvement (<7 points):**
- Model fails to load
- Significant configuration errors

#### Multi-view Image Capture (10 points)

**Excellent (9-10 points):**
- Captures synchronized multi-view images from robot cameras
- Proper viewpoint selection for reconstruction
- Handles camera calibration correctly
- Implements proper ROS2 integration

**Good (7-8 points):**
- Basic multi-view capture working
- Minor synchronization issues
- Some calibration problems

**Needs Improvement (<7 points):**
- Single view only
- Capture failures
- No synchronization

#### 3D Reconstruction Quality (10 points)

**Excellent (9-10 points):**
- Generates dense, accurate 3D reconstructions
- Minimal artifacts
- Handles multiple objects/scenes
- High-quality point cloud output

**Good (7-8 points):**
- Produces usable reconstructions
- Some noise or missing regions
- Generally functional

**Needs Improvement (<7 points):**
- Poor reconstruction quality
- Fails to generate 3D output
- Excessive noise/artifacts

---

### 2. Point Cloud Processing (20 points)

#### Point Cloud Generation (8 points)

**Excellent (7-8 points):**
- Converts MUSt3R output to ROS2 PointCloud2 messages
- Maintains proper coordinate frames
- Efficient conversion pipeline
- Clean integration with ROS2

**Good (5-6 points):**
- Basic point cloud generation
- Minor frame issues
- Works with some limitations

**Needs Improvement (<5 points):**
- Point cloud format errors
- Missing transformations
- Failed integration

#### Filtering & Segmentation (7 points)

**Excellent (6-7 points):**
- Implements voxel filtering
- Removes outliers effectively
- Segments ground plane and obstacles
- Clean, usable output

**Good (4-5 points):**
- Basic filtering implemented
- Some segmentation working
- Partial functionality

**Needs Improvement (<4 points):**
- No filtering
- Segmentation failures
- Unusable output

#### Visualization (5 points)

**Excellent (4-5 points):**
- Real-time point cloud display in RViz2
- Colored by height/distance
- Clear obstacle representation
- Smooth, responsive visualization

**Good (3 points):**
- Basic visualization working
- Some display issues
- Functional but not optimal

**Needs Improvement (<3 points):**
- Unable to visualize
- Significant rendering problems
- Crashes or errors

---

### 3. Vision-based Navigation (15 points)

#### Obstacle Detection (7 points)

**Excellent (6-7 points):**
- Accurately detects obstacles from reconstructed point clouds
- Computes distances and safe zones
- Real-time performance
- Low false positive/negative rate

**Good (4-5 points):**
- Basic obstacle detection
- Occasional false positives/negatives
- Generally functional

**Needs Improvement (<4 points):**
- Poor obstacle detection
- Many failures
- Unreliable

#### Path Planning (8 points)

**Excellent (7-8 points):**
- Implements navigation using 3D reconstruction data
- Robot successfully navigates around obstacles
- Smooth, efficient paths
- Real-time replanning

**Good (5-6 points):**
- Basic navigation working
- Some collision risks
- Needs improvement

**Needs Improvement (<5 points):**
- Unable to navigate
- Frequent collisions
- System fails

---

### 4. Technical Report & Documentation (10 points)

#### Implementation Details (5 points)

**Excellent (4-5 points):**
- Clear explanation of MUSt3R integration
- Point cloud processing pipeline documented
- Navigation strategy explained
- Code well-documented

**Good (3 points):**
- Good documentation with minor gaps
- Most details covered
- Some areas need more explanation

**Needs Improvement (<3 points):**
- Poor or missing documentation
- Unclear implementation
- Hard to understand

#### Results & Analysis (5 points)

**Excellent (4-5 points):**
- Quantitative metrics (reconstruction accuracy, navigation success rate)
- Comparison with baseline or other methods
- Thorough analysis of results
- Identifies limitations and future work

**Good (3 points):**
- Basic results presented
- Limited analysis
- Some metrics included

**Needs Improvement (<3 points):**
- Missing results
- No analysis
- Incomplete evaluation

---

## Reference: MUSt3R

### What is MUSt3R?

**MUSt3R** (Multi-view Stereo 3D Reconstruction) is a CVPR 2025 paper from NAVER Labs that performs dense 3D reconstruction from multiple views.

- **Paper**: CVPR 2025 (arXiv:2503.01661)
- **GitHub**: https://github.com/naver/must3r
- **Model**: naver/MUSt3R_ViTLarge_BaseDecoder_512_dpt

### Key Concepts

1. **Multi-view Stereo**: Reconstructs 3D scene from multiple 2D images
2. **Dense Reconstruction**: Produces detailed 3D point clouds
3. **Deep Learning**: Uses Vision Transformer (ViT) backbone
4. **Pointmaps**: Outputs 3D coordinates for each pixel

### Important: MUSt3R vs MASt3R

- **MUSt3R** (this assignment): Multi-view Stereo 3D Reconstruction
- **MASt3R**: Different model focused on matching and tracking
- Use the **correct** repository: https://github.com/naver/must3r

---

## Implementation Requirements

### Core MUSt3R Implementation

```python
# Required MUSt3R setup
from must3r import MUST3R
import torch

class MUST3RProcessor:
    def __init__(self):
        self.model = MUST3R.from_pretrained(
            "naver/MUSt3R_ViTLarge_BaseDecoder_512_dpt"
        ).to('cuda' if torch.cuda.is_available() else 'cpu')

    def reconstruct_scene(self, images):
        # Multi-view 3D reconstruction
        pointmaps, confidences = self.model(images)
        return self.process_pointcloud(pointmaps)
```

### ROS2 Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

class MUST3RNode(Node):
    def __init__(self):
        super().__init__('must3r_node')

        # Initialize MUSt3R processor
        self.processor = MUST3RProcessor()

        # Subscribers for camera images
        self.image_sub = self.create_subscription(
            Image, '/zed/left/image_rect_color',
            self.image_callback, 10)

        # Publisher for point clouds
        self.pc_pub = self.create_publisher(
            PointCloud2, '/must3r/point_cloud', 10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS image to numpy
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

        # Run reconstruction
        point_cloud = self.processor.reconstruct_scene([cv_image])

        # Publish point cloud
        pc_msg = self.create_pointcloud2_msg(point_cloud)
        self.pc_pub.publish(pc_msg)
```

---

## ZED Camera Integration

### Step 1: Install ZED SDK 5.1

Visit: https://www.stereolabs.com/developers/release

Download and install ZED SDK 5.1 for your Ubuntu version:

```bash
# For Ubuntu 22.04
wget https://download.stereolabs.com/zedsdk/5.1/cu121/ubuntu22
chmod +x ubuntu22
./ubuntu22

# Follow installation prompts
# Choose: Install CUDA (if not already installed)
# Choose: Install Python API
```

### Step 2: Install ZED ROS2 Wrapper

```bash
# Create workspace if it doesn't exist
mkdir -p ~/ros2_ws/src/
cd ~/ros2_ws/src/

# Clone ZED ROS2 wrapper
git clone https://github.com/stereolabs/zed-ros2-wrapper.git

cd ~/ros2_ws

# Update package index
sudo apt update

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

# Source the installation
echo "source $(pwd)/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Test ZED Camera

```bash
# Launch ZED node
ros2 launch zed_wrapper zed_camera.launch.py

# In another terminal, check topics
ros2 topic list | grep zed

# You should see:
# /zed/left/image_rect_color
# /zed/right/image_rect_color
# /zed/depth/depth_registered
# /zed/point_cloud/cloud_registered
# etc.

# View images in RViz2
rviz2
# Add Image display, topic: /zed/left/image_rect_color
```

### Step 4: ZED Configuration for MUSt3R

Create `config/zed_must3r.yaml`:

```yaml
# ZED camera configuration for MUSt3R
camera:
  resolution: 2  # 1080p (1920x1080)
  grab_frame_rate: 15

depth:
  quality: 1  # PERFORMANCE mode
  sensing_mode: 0  # STANDARD
  depth_stabilization: true

video:
  img_downsample_factor: 2.0  # Downsample to ~512x512 for MUSt3R

point_cloud:
  mapping_enabled: false
  resolution: 0.05  # 5cm
```

---

## Environment Setup

### Python Environment

```bash
# Create Python 3.11 virtual environment
sudo apt install python3.11 python3.11-venv
python3.11 -m venv ~/must3r_env
source ~/must3r_env/bin/activate

# Install PyTorch 2.7.0 with CUDA support
pip install torch==2.7.0 torchvision==0.18.0 --index-url https://download.pytorch.org/whl/cu121

# Install MUSt3R
pip install must3r@git+https://github.com/naver/must3r.git

# Install additional dependencies
pip install opencv-python numpy scipy open3d

# Install ROS2 Python packages
pip install rospkg transforms3d
```

### Verify Installation

```python
# test_must3r.py
import torch
from must3r import MUST3R

print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")

model = MUST3R.from_pretrained("naver/MUSt3R_ViTLarge_BaseDecoder_512_dpt")
print("MUSt3R model loaded successfully!")
```

---

## Submission Requirements

Submit to Gradescope by **December 9, 2025 @ 11:59 PM**:

### 1. Code Package (must3r_nav/)

```
must3r_nav/
├── must3r_nav/
│   ├── __init__.py
│   ├── must3r_processor.py      # Core MUSt3R wrapper
│   ├── must3r_node.py            # ROS2 node
│   ├── point_cloud_processor.py  # Filtering/segmentation
│   ├── obstacle_detector.py      # Vision-based detection
│   └── navigator.py              # Path planning
├── launch/
│   ├── zed_must3r.launch.py     # ZED + MUSt3R
│   └── full_system.launch.py    # Complete system
├── config/
│   └── zed_must3r.yaml
├── package.xml
├── setup.py
└── README.md
```

### 2. Demo Video (3 minutes)

Show the following:
1. ZED camera capturing multi-view images
2. MUSt3R generating 3D point clouds in real-time
3. Point cloud visualization in RViz2
4. Obstacle detection from reconstructed point clouds
5. Robot navigating autonomously using vision-based navigation

Upload to YouTube (unlisted) and include link in submission.

### 3. ROSbag Recording

Record all relevant topics:
```bash
ros2 bag record -o must3r_demo \
  /zed/left/image_rect_color \
  /zed/right/image_rect_color \
  /must3r/point_cloud \
  /obstacles/detected \
  /cmd_vel \
  /tf \
  /joint_states
```

Minimum 5 minutes of operation showing full system functionality.

### 4. Technical Report (4 pages, IEEE format)

**Structure:**
- **Abstract**: Problem, approach, results (150 words)
- **Introduction**: Motivation for 3D reconstruction in robotics
- **Related Work**: MUSt3R paper, other 3D reconstruction methods
- **Methodology**:
  - MUSt3R integration
  - Point cloud processing pipeline
  - Vision-based navigation algorithm
- **Results**:
  - Reconstruction quality metrics
  - Navigation success rate
  - Computational performance
- **Discussion**: Limitations, challenges, future work
- **Conclusion**: Summary of achievements

**IEEE Template**: https://www.ieee.org/conferences/publishing/templates.html

---

## Evaluation Criteria

### Performance Metrics

- **Reconstruction Accuracy**: Compare 3D reconstruction with ground truth (if available)
- **Point Cloud Density**: Points per cubic meter
- **Processing Time**: FPS for reconstruction
- **Obstacle Detection Accuracy**: Precision/recall for obstacle detection
- **Navigation Success Rate**: Percentage of successful obstacle avoidance

### Code Quality

- Clear documentation and comments
- Modular design with ROS2 best practices
- Proper error handling
- Git commit history showing progress
- Type hints and docstrings

---

## Resources

### MUSt3R Resources

- **GitHub**: https://github.com/naver/must3r
- **Paper**: CVPR 2025 (arXiv:2503.01661)
- **Model Card**: https://huggingface.co/naver/MUSt3R_ViTLarge_BaseDecoder_512_dpt

### ZED Camera Resources

- **ZED SDK Documentation**: https://www.stereolabs.com/docs/
- **ZED ROS2 Wrapper**: https://github.com/stereolabs/zed-ros2-wrapper
- **ZED Tutorials**: https://www.stereolabs.com/docs/ros2/

### ROS2 Resources

- **Point Cloud Processing**: http://wiki.ros.org/pcl_ros
- **RViz2 Visualization**: https://github.com/ros2/rviz
- **Image Transport**: https://github.com/ros-perception/image_transport_plugins

### Course Resources

- **Course Website**: https://kulbir-singh-ahluwalia.com/cs498gc/fa25/
- **Office Hours**: Wed 1:30-2:30 PM @ SC 4407
- **Campuswire**: Tag posts with `#assignment4-part2`

---

## Common Issues & Solutions

### Issue: MUSt3R Model Fails to Load

**Solution**: Check PyTorch and CUDA compatibility
```bash
python -c "import torch; print(torch.cuda.is_available())"
pip install --upgrade torch torchvision
```

### Issue: ZED Camera Not Detected

**Solution**: Verify ZED SDK installation
```bash
/usr/local/zed/tools/ZED_Diagnostic
```

### Issue: Point Cloud Not Displaying in RViz2

**Solution**: Check frame IDs and TF tree
```bash
ros2 topic echo /must3r/point_cloud --once
ros2 run tf2_tools view_frames
```

### Issue: Slow Reconstruction Performance

**Solution**:
- Reduce image resolution to 512x512
- Use GPU acceleration
- Downsample point cloud output
- Reduce reconstruction frequency

---

## Grading Notes

- **Late Penalty**: -10% per day (max 3 days)
- **Partial Credit**: Available for each component
- **Bonus Points**: +5 for exceptional visualization or novel navigation approaches
- **Academic Integrity**: Code must be your own implementation

---

## Important Reminders

1. This is **MUSt3R 3D reconstruction**, NOT 2D SLAM
2. Input is **RGB images**, not laser scans
3. Output is **3D point clouds**, not 2D occupancy grids
4. Use **https://github.com/naver/must3r** (correct repository)
5. Install **ZED camera** for multi-view capture
6. Use **Python 3.11** with **PyTorch 2.7.0**

---

**Good luck with your 3D reconstruction implementation! Start early, test the ZED camera first, and verify MUSt3R works before integration.**

*Last updated: December 4, 2025*
