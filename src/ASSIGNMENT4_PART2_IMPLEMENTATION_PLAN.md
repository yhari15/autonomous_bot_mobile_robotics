# Assignment 4 Part 2: Implementation Plan
## Detailed Task Breakdown and Timeline

**Project**: Mobile Manipulation with SLAM
**Due**: December 9, 2025 @ 11:00 PM
**Current Date**: December 1, 2025
**Time Remaining**: 8 days

---

## Executive Summary

This implementation plan breaks down Assignment 4 Part 2 into manageable, sequential tasks. The project builds a complete autonomous mobile manipulation system with SLAM capabilities.

### High-Level Phases

1. **Phase 1**: Sensor Integration & EKF Fusion (Days 1-2)
2. **Phase 2**: SLAM Setup & Testing (Days 2-3)
3. **Phase 3**: Navigation Stack Configuration (Days 3-4)
4. **Phase 4**: Autonomous Exploration (Days 4-5)
5. **Phase 5**: Manipulation Integration (Days 5-6)
6. **Phase 6**: Mars Environment (Days 6-7, optional)
7. **Phase 7**: Testing & Documentation (Days 7-8)

---

## Phase 1: Sensor Integration & EKF Fusion

### Day 1 Morning: Add Sensors to Robot Model

#### Task 1.1: Verify Existing Sensors
```bash
# Check what sensors are already in the URDF
cd ~/ros2_ws/src/husky_ur3_description/urdf
grep -n "sensor" husky_ur3_gripper.urdf.xacro
grep -n "laser" husky.urdf.xacro
```

**Expected**: Laser scanner should already exist in Husky base
**Action**: Document existing sensor configuration

#### Task 1.2: Add IMU Sensor Plugin

**File**: `husky_ur3_description/urdf/sensors.urdf.xacro` (create if doesn't exist)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- IMU Sensor -->
  <gazebo reference="base_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100.0</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.1</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.1</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.1</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>

  <!-- GPS Sensor -->
  <link name="gps_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>

  <joint name="gps_joint" type="fixed">
    <parent link="top_plate_link"/>
    <child link="gps_link"/>
    <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
  </joint>

  <gazebo reference="gps_link">
    <sensor name="gps_sensor" type="gps">
      <always_on>true</always_on>
      <update_rate>4.0</update_rate>
      <gps>
        <position_sensing>
          <horizontal>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2.0</stddev>
            </noise>
          </horizontal>
          <vertical>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>4.0</stddev>
            </noise>
          </vertical>
        </position_sensing>
      </gps>
      <plugin name="gps_plugin" filename="libgazebo_ros_gps_sensor.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:=gps/fix</remapping>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

**Include in main URDF**:
```xml
<!-- In husky_ur3_gripper.urdf.xacro -->
<xacro:include filename="$(find husky_ur3_description)/urdf/sensors.urdf.xacro"/>
```

#### Task 1.3: Test Sensor Data

```bash
# Rebuild workspace
cd ~/ros2_ws
colcon build --packages-select husky_ur3_description
source install/setup.bash

# Launch simulation
ros2 launch husky_ur3_description gazebo_sim.launch.py

# In separate terminals, verify sensors:
ros2 topic list | grep -E "imu|gps|scan"
ros2 topic echo /imu/data --once
ros2 topic echo /gps/fix --once
ros2 topic echo /scan --once

# Check rates
ros2 topic hz /imu/data  # Should be ~100 Hz
ros2 topic hz /gps/fix   # Should be ~4 Hz
ros2 topic hz /scan      # Should be ~10 Hz
```

**Checkpoint**: All three sensors publishing valid data

---

### Day 1 Afternoon: Create EKF Package Structure

#### Task 1.4: Create ROS2 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create husky_ur3_slam \
  --build-type ament_cmake \
  --dependencies rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros

# Create directory structure
cd husky_ur3_slam
mkdir -p launch config src scripts rviz maps
```

#### Task 1.5: Create EKF Configuration

**File**: `config/ekf_params.yaml`

```yaml
ekf_filter_node:
  ros__parameters:
    use_sim_time: true

    frequency: 10.0
    sensor_timeout: 0.5
    two_d_mode: true

    transform_time_offset: 0.0
    transform_timeout: 0.0

    print_diagnostics: true
    debug: false

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # Odometry configuration
    odom0: /odometry/wheel
    odom0_config: [true,  true,  false,  # x, y, z
                   false, false, true,   # roll, pitch, yaw
                   true,  true,  false,  # vx, vy, vz
                   false, false, true,   # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az
    odom0_queue_size: 10
    odom0_nodelay: false
    odom0_differential: false
    odom0_relative: false

    # IMU configuration
    imu0: /imu/data
    imu0_config: [false, false, false,  # x, y, z
                  true,  true,  true,   # roll, pitch, yaw
                  false, false, false,  # vx, vy, vz
                  true,  true,  true,   # vroll, vpitch, vyaw
                  true,  true,  true]   # ax, ay, az
    imu0_queue_size: 10
    imu0_nodelay: false
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true

    # GPS configuration
    navsat0: /gps/fix
    navsat0_config: [true,  true,  true,   # x, y, z
                     false, false, false,  # roll, pitch, yaw
                     false, false, false,  # vx, vy, vz
                     false, false, false,  # vroll, vpitch, vyaw
                     false, false, false]  # ax, ay, az
    navsat0_queue_size: 10
    navsat0_nodelay: false
    navsat0_differential: false

    # Process noise covariance
    process_noise_covariance: [0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015]

    # Initial state covariance
    initial_estimate_covariance: [1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,
                                  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9]
```

#### Task 1.6: Create EKF Launch File

**File**: `launch/ekf_fusion.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('husky_ur3_slam')
    ekf_params = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Robot Localization EKF node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'robot_pose')]
        ),

        # Optional: Navsat transform for GPS
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_params, {'use_sim_time': use_sim_time}],
            remappings=[('gps/fix', 'gps/fix'),
                       ('gps/filtered', 'gps/filtered'),
                       ('odometry/gps', 'odometry/gps')]
        ),
    ])
```

#### Task 1.7: Install Dependencies & Build

```bash
# Install robot_localization
sudo apt install ros-humble-robot-localization

# Build package
cd ~/ros2_ws
colcon build --packages-select husky_ur3_slam
source install/setup.bash
```

---

### Day 1 Evening: Test EKF

#### Task 1.8: Run Full System Test

```bash
# Terminal 1: Launch Gazebo with robot
ros2 launch husky_ur3_description gazebo_sim.launch.py

# Terminal 2: Launch EKF
ros2 launch husky_ur3_slam ekf_fusion.launch.py

# Terminal 3: Monitor EKF output
ros2 topic echo /robot_pose --once
ros2 topic hz /robot_pose  # Should be ~10 Hz

# Terminal 4: Visualize in RViz
rviz2
```

**RViz Configuration**:
1. Fixed Frame: `odom`
2. Add → TF
3. Add → Odometry → Topic: `/robot_pose`
4. Add → Imu → Topic: `/imu/data`

#### Task 1.9: Test with Movement

```bash
# Terminal 5: Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Drive robot around and observe:
# - /robot_pose updates smoothly
# - TF tree shows map → odom → base_link
# - No jumps or discontinuities
```

**Checkpoint**: EKF publishes smooth pose estimates at ≥10 Hz

---

## Phase 2: SLAM Setup & Testing

### Day 2 Morning: Install and Configure SLAM

#### Task 2.1: Install SLAM Packages

```bash
# Install both options to compare
sudo apt install ros-humble-slam-toolbox ros-humble-gmapping

# Additional dependencies
sudo apt install ros-humble-nav2-map-server
```

#### Task 2.2: Create SLAM Configuration

**File**: `config/slam_toolbox_params.yaml`

```yaml
slam_toolbox:
  ros__parameters:
    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping

    # if you'd like to immediately start continuing a map at a given pose
    # or at the dock, but they are mutually exclusive, if pose is given
    # will use pose
    #map_file_name: /path/to/map
    #map_start_pose: [0.0, 0.0, 0.0]
    #map_start_at_dock: true

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 1.0
    resolution: 0.05
    max_laser_range: 25.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000

    # General Parameters
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation Parameters - Correlation Parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # Correlation Parameters - Loop Closure Parameters
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan Matcher Parameters
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0

    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

#### Task 2.3: Create SLAM Launch File

**File**: `launch/slam.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('husky_ur3_slam')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
        ),
    ])
```

#### Task 2.4: Build and Test SLAM

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select husky_ur3_slam
source install/setup.bash

# Test
# Terminal 1: Simulation
ros2 launch husky_ur3_description gazebo_sim.launch.py

# Terminal 2: EKF
ros2 launch husky_ur3_slam ekf_fusion.launch.py

# Terminal 3: SLAM
ros2 launch husky_ur3_slam slam.launch.py

# Terminal 4: RViz
rviz2

# Terminal 5: Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**RViz Setup**:
1. Fixed Frame: `map`
2. Add → Map → Topic: `/map`
3. Add → LaserScan → Topic: `/scan`
4. Add → TF
5. Add → RobotModel

#### Task 2.5: Drive and Build Map

- Drive robot around the warehouse environment
- Ensure map builds correctly
- Check for loop closures when returning to start
- Verify no major drift

**Checkpoint**: Clean 2D map with clear obstacle boundaries

---

### Day 2 Afternoon: Map Saving and Tuning

#### Task 2.6: Save Map

```bash
# While SLAM is running, save the map
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/husky_ur3_slam/maps/warehouse_map

# This creates:
# - warehouse_map.pgm (image)
# - warehouse_map.yaml (metadata)
```

#### Task 2.7: Verify Map Quality

```bash
# View map
eog ~/ros2_ws/src/husky_ur3_slam/maps/warehouse_map.pgm

# Check map parameters
cat ~/ros2_ws/src/husky_ur3_slam/maps/warehouse_map.yaml
```

**Quality Checklist**:
- [ ] Walls are continuous (not broken)
- [ ] Obstacles are clearly defined
- [ ] Resolution is 0.05m or better
- [ ] Free space is white, obstacles are black
- [ ] Unknown areas are gray

#### Task 2.8: Create RViz Config

Save RViz configuration for future use:

**File**: `rviz/slam_config.rviz`

```bash
# In RViz: File → Save Config As
# Save to: ~/ros2_ws/src/husky_ur3_slam/rviz/slam_config.rviz
```

---

## Phase 3: Navigation Stack Configuration

### Day 3: Nav2 Setup

#### Task 3.1: Install Nav2

```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander
```

#### Task 3.2: Configure Costmaps

**File**: `config/costmap_common.yaml`

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: True
  observation_sources: scan
  scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: True
    marking: True
    data_type: "LaserScan"
    raytrace_max_range: 3.0
    raytrace_min_range: 0.0
    obstacle_max_range: 2.5
    obstacle_min_range: 0.0

inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0
  inflation_radius: 0.8

always_send_full_costmap: True
```

**File**: `config/global_costmap.yaml`

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      footprint: "[[0.45, 0.35], [0.45, -0.35], [-0.45, -0.35], [-0.45, 0.35]]"
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.8
      always_send_full_costmap: True
```

**File**: `config/local_costmap.yaml`

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      footprint: "[[0.45, 0.35], [0.45, -0.35], [-0.45, -0.35], [-0.45, 0.35]]"
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          obstacle_max_range: 2.5
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.8
      always_send_full_costmap: True
```

#### Task 3.3: Configure Nav2 Parameters

**File**: `config/nav2_params.yaml`

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_updated_goal_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: -0.5
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 1.0
      decel_lim_x: -0.5
      decel_lim_y: 0.0
      decel_lim_theta: -1.0
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

robot_state_publisher:
  ros__parameters:
    use_sim_time: True

waypoint_follower:
  ros__parameters:
    use_sim_time: True
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200

velocity_smoother:
  ros__parameters:
    use_sim_time: True
    smoothing_frequency: 20.0
    scale_velocities: False
    feedback: "OPEN_LOOP"
    max_velocity: [0.5, 0.0, 1.0]
    min_velocity: [-0.5, 0.0, -1.0]
    max_accel: [0.5, 0.0, 1.0]
    max_decel: [-0.5, 0.0, -1.0]
    odom_topic: "odometry/filtered"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
```

This is getting quite long. Should I continue with the rest of the implementation plan (Days 4-8), or would you like me to summarize the remaining phases at a higher level?

The plan so far covers:
- ✅ Day 1: Sensor integration + EKF setup
- ✅ Day 2: SLAM configuration
- ✅ Day 3: Nav2 setup (in progress)

Remaining to document:
- Day 4-5: Autonomous exploration + manipulation
- Day 6-7: Mars environment (bonus)
- Day 7-8: Testing and documentation
