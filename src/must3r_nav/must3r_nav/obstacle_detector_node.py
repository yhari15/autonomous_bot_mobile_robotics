#!/usr/bin/env python3
"""
Obstacle Detector Node
Subscribes to MUSt3R point cloud, processes it, and publishes:
- Filtered point cloud
- Ground plane (green)
- Obstacles (red)
- Obstacle markers for visualization

Assignment 4 Part 2 - Vision-based Navigation (15 points)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import struct

from .point_cloud_processor import PointCloudProcessor


class ObstacleDetectorNode(Node):
    """
    Detects obstacles from MUSt3R point cloud output
    """
    
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Parameters
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('ground_threshold', 0.15)
        self.declare_parameter('min_obstacle_height', 0.1)
        self.declare_parameter('max_obstacle_height', 2.0)
        self.declare_parameter('cluster_tolerance', 0.15)
        self.declare_parameter('min_cluster_size', 10)
        
        # Get parameters
        voxel_size = self.get_parameter('voxel_size').value
        ground_threshold = self.get_parameter('ground_threshold').value
        min_obstacle_height = self.get_parameter('min_obstacle_height').value
        max_obstacle_height = self.get_parameter('max_obstacle_height').value
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        
        # Initialize processor
        self.processor = PointCloudProcessor(
            voxel_size=voxel_size,
            ground_threshold=ground_threshold,
            min_obstacle_height=min_obstacle_height,
            max_obstacle_height=max_obstacle_height
        )
        
        # Subscribers
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/must3r/point_cloud',
            self.pointcloud_callback,
            10
        )
        
        # Publishers
        self.filtered_pub = self.create_publisher(
            PointCloud2, '/obstacles/filtered', 10
        )
        self.ground_pub = self.create_publisher(
            PointCloud2, '/obstacles/ground', 10
        )
        self.obstacles_pub = self.create_publisher(
            PointCloud2, '/obstacles/points', 10
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, '/obstacles/markers', 10
        )
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Obstacle Detector Node Started")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Voxel size: {voxel_size}m")
        self.get_logger().info(f"Ground threshold: {ground_threshold}m")
        self.get_logger().info(f"Obstacle height range: {min_obstacle_height}-{max_obstacle_height}m")
        self.get_logger().info("")
        self.get_logger().info("Subscribed to: /must3r/point_cloud")
        self.get_logger().info("Publishing:")
        self.get_logger().info("  /obstacles/filtered (white)")
        self.get_logger().info("  /obstacles/ground (green)")
        self.get_logger().info("  /obstacles/points (red)")
        self.get_logger().info("  /obstacles/markers (bounding boxes)")
        self.get_logger().info("=" * 60)
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud"""
        try:
            # Convert PointCloud2 to numpy array
            points = self.pointcloud2_to_array(msg)
            
            if len(points) == 0:
                return
            
            # Process point cloud
            result = self.processor.process(points)
            
            # Log stats
            stats = result['stats']
            self.get_logger().info(
                f"Processed: {stats['raw']} → {stats['filtered']} filtered, "
                f"{stats['ground']} ground, {stats['obstacles']} obstacles"
            )
            
            # Publish filtered - colored by HEIGHT (gradient: blue=low → red=high)
            if len(result['filtered']) > 0:
                self.publish_height_colored_pointcloud(
                    result['filtered'], 
                    self.filtered_pub,
                    msg.header.frame_id
                )
            
            # Publish ground (green gradient by distance from robot)
            if len(result['ground']) > 0:
                self.publish_distance_colored_pointcloud(
                    result['ground'],
                    self.ground_pub,
                    msg.header.frame_id,
                    base_color=(0, 200, 0)  # Green
                )
            
            # Publish obstacles - colored by HEIGHT (gradient: yellow=low → red=high)
            if len(result['obstacles']) > 0:
                self.publish_height_colored_pointcloud(
                    result['obstacles'],
                    self.obstacles_pub,
                    msg.header.frame_id,
                    color_scheme='obstacle'  # Yellow to red
                )
                
                # Detect and publish obstacle clusters
                obstacles = self.processor.detect_obstacle_clusters(
                    result['obstacles'],
                    cluster_tolerance=self.cluster_tolerance,
                    min_cluster_size=self.min_cluster_size
                )
                
                if obstacles:
                    self.publish_obstacle_markers(obstacles, msg.header.frame_id)
                    self.get_logger().info(f"Detected {len(obstacles)} obstacle clusters")
                    
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def pointcloud2_to_array(self, msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 message to numpy array"""
        point_step = msg.point_step
        num_points = msg.width * msg.height
        
        if num_points == 0:
            return np.array([]).reshape(0, 3)
        
        points = np.zeros((num_points, 3), dtype=np.float32)
        
        for i in range(num_points):
            offset = i * point_step
            x = struct.unpack_from('f', msg.data, offset)[0]
            y = struct.unpack_from('f', msg.data, offset + 4)[0]
            z = struct.unpack_from('f', msg.data, offset + 8)[0]
            points[i] = [x, y, z]
        
        # Filter out invalid points (NaN, Inf)
        valid_mask = np.isfinite(points).all(axis=1)
        return points[valid_mask]
    
    def height_to_color(self, z: float, min_z: float, max_z: float, 
                        scheme: str = 'default') -> tuple:
        """
        Convert height to RGB color using gradient
        
        Schemes:
        - 'default': Blue (low) → Cyan → Green → Yellow → Red (high)
        - 'obstacle': Yellow (low) → Orange → Red (high)
        - 'ground': Dark green (low) → Light green (high)
        """
        # Normalize height to 0-1
        if max_z - min_z > 0.01:
            t = np.clip((z - min_z) / (max_z - min_z), 0, 1)
        else:
            t = 0.5
        
        if scheme == 'obstacle':
            # Yellow → Orange → Red
            r = 255
            g = int(255 * (1 - t))
            b = 0
        elif scheme == 'ground':
            # Dark green → Light green
            r = int(50 + 100 * t)
            g = int(150 + 105 * t)
            b = int(50 + 50 * t)
        else:
            # Rainbow: Blue → Cyan → Green → Yellow → Red
            if t < 0.25:
                # Blue to Cyan
                r = 0
                g = int(255 * t * 4)
                b = 255
            elif t < 0.5:
                # Cyan to Green
                r = 0
                g = 255
                b = int(255 * (1 - (t - 0.25) * 4))
            elif t < 0.75:
                # Green to Yellow
                r = int(255 * (t - 0.5) * 4)
                g = 255
                b = 0
            else:
                # Yellow to Red
                r = 255
                g = int(255 * (1 - (t - 0.75) * 4))
                b = 0
        
        return (r, g, b)
    
    def distance_to_color(self, dist: float, max_dist: float, 
                          base_color: tuple) -> tuple:
        """
        Color by distance - brighter when closer
        """
        t = np.clip(dist / max_dist, 0, 1)
        # Closer = brighter, farther = darker
        factor = 1.0 - 0.6 * t
        return (
            int(base_color[0] * factor),
            int(base_color[1] * factor),
            int(base_color[2] * factor)
        )
    
    def publish_height_colored_pointcloud(self, points: np.ndarray, publisher,
                                          frame_id: str, color_scheme: str = 'default'):
        """Publish point cloud colored by height (z-coordinate)"""
        if len(points) == 0:
            return
            
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        
        # XYZRGB format
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        # Get height range
        z_coords = points[:, 2]
        min_z = np.min(z_coords)
        max_z = np.max(z_coords)
        
        point_struct = struct.Struct('fffI')
        buffer = bytearray(point_struct.size * len(points))
        
        for i, point in enumerate(points):
            r, g, b = self.height_to_color(point[2], min_z, max_z, color_scheme)
            rgb = (int(r) << 16) | (int(g) << 8) | int(b)
            point_struct.pack_into(buffer, i * point_struct.size, 
                                   float(point[0]), float(point[1]), float(point[2]), rgb)
        
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
        
        publisher.publish(pc_msg)
    
    def publish_distance_colored_pointcloud(self, points: np.ndarray, publisher,
                                            frame_id: str, base_color: tuple):
        """Publish point cloud colored by distance from robot (origin)"""
        if len(points) == 0:
            return
            
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        
        # XYZRGB format
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        # Compute distances
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        max_dist = np.max(distances) if np.max(distances) > 0 else 1.0
        
        point_struct = struct.Struct('fffI')
        buffer = bytearray(point_struct.size * len(points))
        
        for i, point in enumerate(points):
            r, g, b = self.distance_to_color(distances[i], max_dist, base_color)
            rgb = (int(r) << 16) | (int(g) << 8) | int(b)
            point_struct.pack_into(buffer, i * point_struct.size, 
                                   float(point[0]), float(point[1]), float(point[2]), rgb)
        
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
        
        publisher.publish(pc_msg)
    
    def publish_colored_pointcloud(self, points: np.ndarray, publisher,
                                    frame_id: str, color: tuple):
        """Publish point cloud with single RGB color (for backwards compatibility)"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        
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
            point_struct.pack_into(buffer, i * point_struct.size, 
                                   float(point[0]), float(point[1]), float(point[2]), rgb)
        
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
        
        publisher.publish(pc_msg)
    
    def publish_obstacle_markers(self, obstacles: list, frame_id: str):
        """Publish obstacle bounding boxes as markers"""
        marker_array = MarkerArray()
        
        for i, obs in enumerate(obstacles):
            # Bounding box marker
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Position at centroid
            marker.pose.position.x = float(obs['centroid'][0])
            marker.pose.position.y = float(obs['centroid'][1])
            marker.pose.position.z = float(obs['centroid'][2])
            marker.pose.orientation.w = 1.0
            
            # Size from bounding box
            marker.scale.x = float(max(obs['size'][0], 0.1))
            marker.scale.y = float(max(obs['size'][1], 0.1))
            marker.scale.z = float(max(obs['size'][2], 0.1))
            
            # Red semi-transparent
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            
            marker.lifetime.sec = 1  # Expire after 1 second
            
            marker_array.markers.append(marker)
            
            # Text label with distance
            if 'distance' in obs:
                text_marker = Marker()
                text_marker.header = marker.header
                text_marker.ns = "obstacle_labels"
                text_marker.id = i
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = float(obs['centroid'][0])
                text_marker.pose.position.y = float(obs['centroid'][1])
                text_marker.pose.position.z = float(obs['centroid'][2]) + 0.5
                text_marker.scale.z = 0.3
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                text_marker.text = f"{obs['distance']:.1f}m"
                text_marker.lifetime.sec = 1
                marker_array.markers.append(text_marker)
        
        self.markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObstacleDetectorNode()
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

