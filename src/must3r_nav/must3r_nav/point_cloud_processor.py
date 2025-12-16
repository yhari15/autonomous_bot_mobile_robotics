#!/usr/bin/env python3
"""
Point Cloud Processor for MUSt3R Output
Implements filtering, segmentation, and obstacle detection

Assignment 4 Part 2 - Point Cloud Processing (20 points)
- Voxel filtering (downsampling)
- Outlier removal
- Ground plane segmentation
- Obstacle segmentation
"""

import numpy as np

# Try to import Open3D for advanced processing
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    print("Warning: Open3D not available. Using basic numpy processing.")


class PointCloudProcessor:
    """
    Processes point clouds from MUSt3R for navigation
    """
    
    def __init__(self,
                 voxel_size: float = 0.05,
                 outlier_nb_neighbors: int = 20,
                 outlier_std_ratio: float = 2.0,
                 ground_threshold: float = 0.15,
                 min_obstacle_height: float = 0.1,
                 max_obstacle_height: float = 2.0):
        """
        Initialize processor with parameters
        
        Args:
            voxel_size: Size of voxel grid for downsampling (meters)
            outlier_nb_neighbors: Number of neighbors for outlier detection
            outlier_std_ratio: Standard deviation ratio for outlier removal
            ground_threshold: Height threshold for ground plane (meters)
            min_obstacle_height: Minimum height to consider as obstacle
            max_obstacle_height: Maximum height to consider as obstacle
        """
        self.voxel_size = voxel_size
        self.outlier_nb_neighbors = outlier_nb_neighbors
        self.outlier_std_ratio = outlier_std_ratio
        self.ground_threshold = ground_threshold
        self.min_obstacle_height = min_obstacle_height
        self.max_obstacle_height = max_obstacle_height
        
    def process(self, points: np.ndarray) -> dict:
        """
        Full processing pipeline
        
        Args:
            points: Nx3 numpy array of XYZ points
            
        Returns:
            Dictionary with processed point clouds:
            - 'filtered': Voxelized and outlier-removed points
            - 'ground': Ground plane points
            - 'obstacles': Obstacle points
            - 'stats': Processing statistics
        """
        if points is None or len(points) == 0:
            return {
                'filtered': np.array([]).reshape(0, 3),
                'ground': np.array([]).reshape(0, 3),
                'obstacles': np.array([]).reshape(0, 3),
                'stats': {'raw': 0, 'filtered': 0, 'ground': 0, 'obstacles': 0}
            }
        
        raw_count = len(points)
        
        # Step 1: Voxel filtering (downsampling)
        filtered = self.voxel_filter(points)
        
        # Step 2: Outlier removal
        filtered = self.remove_outliers(filtered)
        filtered_count = len(filtered)
        
        # Step 3: Ground/obstacle segmentation
        ground, obstacles = self.segment_ground_obstacles(filtered)
        
        return {
            'filtered': filtered,
            'ground': ground,
            'obstacles': obstacles,
            'stats': {
                'raw': raw_count,
                'filtered': filtered_count,
                'ground': len(ground),
                'obstacles': len(obstacles)
            }
        }
    
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
            
        if OPEN3D_AVAILABLE:
            # Use Open3D for efficient voxel filtering
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            return np.asarray(pcd_down.points)
        else:
            # Basic numpy implementation
            # Round to voxel grid and remove duplicates
            voxel_indices = np.floor(points / self.voxel_size).astype(int)
            _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
            return points[unique_indices]
    
    def remove_outliers(self, points: np.ndarray) -> np.ndarray:
        """
        Remove statistical outliers from point cloud
        
        Args:
            points: Nx3 point cloud
            
        Returns:
            Filtered point cloud with outliers removed
        """
        if len(points) < self.outlier_nb_neighbors:
            return points
            
        if OPEN3D_AVAILABLE:
            # Use Open3D for statistical outlier removal
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd_filtered, _ = pcd.remove_statistical_outlier(
                nb_neighbors=self.outlier_nb_neighbors,
                std_ratio=self.outlier_std_ratio
            )
            return np.asarray(pcd_filtered.points)
        else:
            # Basic numpy implementation using distance-based filtering
            from scipy.spatial import KDTree
            tree = KDTree(points)
            
            # Find k nearest neighbors for each point
            distances, _ = tree.query(points, k=self.outlier_nb_neighbors + 1)
            mean_distances = np.mean(distances[:, 1:], axis=1)  # Exclude self
            
            # Remove points with high mean distance (outliers)
            threshold = np.mean(mean_distances) + self.outlier_std_ratio * np.std(mean_distances)
            mask = mean_distances < threshold
            
            return points[mask]
    
    def segment_ground_obstacles(self, points: np.ndarray) -> tuple:
        """
        Segment point cloud into ground plane and obstacles
        
        Uses height-based segmentation:
        - Ground: points below ground_threshold
        - Obstacles: points between min_obstacle_height and max_obstacle_height
        
        Args:
            points: Nx3 point cloud
            
        Returns:
            Tuple of (ground_points, obstacle_points)
        """
        if len(points) == 0:
            empty = np.array([]).reshape(0, 3)
            return empty, empty
        
        # Get Z coordinates (height)
        z_coords = points[:, 2]
        
        # Ground: points near z=0 (below threshold)
        ground_mask = z_coords < self.ground_threshold
        
        # Obstacles: points above ground but below max height
        obstacle_mask = (z_coords >= self.min_obstacle_height) & \
                       (z_coords <= self.max_obstacle_height)
        
        ground_points = points[ground_mask]
        obstacle_points = points[obstacle_mask]
        
        return ground_points, obstacle_points
    
    def detect_obstacle_clusters(self, obstacle_points: np.ndarray, 
                                  cluster_tolerance: float = 0.1,
                                  min_cluster_size: int = 10) -> list:
        """
        Cluster obstacle points into individual obstacles
        
        Args:
            obstacle_points: Nx3 obstacle point cloud
            cluster_tolerance: Maximum distance between points in a cluster
            min_cluster_size: Minimum points to form a cluster
            
        Returns:
            List of obstacle dictionaries with 'points', 'centroid', 'bbox'
        """
        if len(obstacle_points) < min_cluster_size:
            return []
        
        obstacles = []
        
        if OPEN3D_AVAILABLE:
            # Use Open3D DBSCAN clustering
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(obstacle_points)
            
            labels = np.array(pcd.cluster_dbscan(
                eps=cluster_tolerance,
                min_points=min_cluster_size
            ))
            
            max_label = labels.max()
            
            for i in range(max_label + 1):
                cluster_mask = labels == i
                cluster_points = obstacle_points[cluster_mask]
                
                if len(cluster_points) >= min_cluster_size:
                    centroid = np.mean(cluster_points, axis=0)
                    min_bound = np.min(cluster_points, axis=0)
                    max_bound = np.max(cluster_points, axis=0)
                    
                    obstacles.append({
                        'points': cluster_points,
                        'centroid': centroid,
                        'min_bound': min_bound,
                        'max_bound': max_bound,
                        'size': max_bound - min_bound,
                        'num_points': len(cluster_points)
                    })
        else:
            # Basic clustering using grid-based approach
            # Group points by XY position
            grid_size = cluster_tolerance * 2
            grid_indices = np.floor(obstacle_points[:, :2] / grid_size).astype(int)
            
            unique_cells = np.unique(grid_indices, axis=0)
            
            for cell in unique_cells:
                mask = np.all(grid_indices == cell, axis=1)
                cluster_points = obstacle_points[mask]
                
                if len(cluster_points) >= min_cluster_size:
                    centroid = np.mean(cluster_points, axis=0)
                    min_bound = np.min(cluster_points, axis=0)
                    max_bound = np.max(cluster_points, axis=0)
                    
                    obstacles.append({
                        'points': cluster_points,
                        'centroid': centroid,
                        'min_bound': min_bound,
                        'max_bound': max_bound,
                        'size': max_bound - min_bound,
                        'num_points': len(cluster_points)
                    })
        
        return obstacles
    
    def compute_obstacle_distances(self, obstacles: list, 
                                    robot_position: np.ndarray = None) -> list:
        """
        Compute distances from robot to each obstacle
        
        Args:
            obstacles: List of obstacle dictionaries
            robot_position: XYZ position of robot (default: origin)
            
        Returns:
            List of obstacles with 'distance' field added
        """
        if robot_position is None:
            robot_position = np.array([0.0, 0.0, 0.0])
        
        for obs in obstacles:
            centroid = obs['centroid']
            # 2D distance (XY plane)
            distance = np.sqrt(
                (centroid[0] - robot_position[0])**2 + 
                (centroid[1] - robot_position[1])**2
            )
            obs['distance'] = distance
        
        # Sort by distance
        obstacles.sort(key=lambda x: x['distance'])
        
        return obstacles


# Test if run directly
if __name__ == "__main__":
    print("Point Cloud Processor Test")
    print(f"Open3D available: {OPEN3D_AVAILABLE}")
    
    # Generate test point cloud
    np.random.seed(42)
    
    # Ground points (z near 0)
    ground = np.random.rand(500, 3) * [10, 10, 0.1]
    
    # Obstacle points (elevated)
    obstacle1 = np.random.rand(100, 3) * [1, 1, 1] + [3, 3, 0.5]
    obstacle2 = np.random.rand(80, 3) * [0.5, 0.5, 0.8] + [-2, 4, 0.3]
    
    # Outliers (random high points)
    outliers = np.random.rand(20, 3) * [10, 10, 5] + [0, 0, 3]
    
    # Combine
    test_points = np.vstack([ground, obstacle1, obstacle2, outliers])
    
    # Process
    processor = PointCloudProcessor()
    result = processor.process(test_points)
    
    print(f"\nProcessing Results:")
    print(f"  Raw points: {result['stats']['raw']}")
    print(f"  After filtering: {result['stats']['filtered']}")
    print(f"  Ground points: {result['stats']['ground']}")
    print(f"  Obstacle points: {result['stats']['obstacles']}")
    
    # Detect obstacle clusters
    obstacles = processor.detect_obstacle_clusters(result['obstacles'])
    print(f"\nDetected {len(obstacles)} obstacle clusters:")
    for i, obs in enumerate(obstacles):
        print(f"  Obstacle {i+1}: {obs['num_points']} points at {obs['centroid']}")

















