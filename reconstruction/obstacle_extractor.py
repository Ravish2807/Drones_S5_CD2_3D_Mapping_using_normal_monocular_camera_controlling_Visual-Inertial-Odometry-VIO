#!/usr/bin/env python3
"""
3D Obstacle Extraction & Spatial Clustering Engine.
Segments occupied 3D voxel space into distinct obstacle entities using Euclidean DBSCAN clustering.
Computes oriented bounding boxes, collision safety margins, and proximity risks for autonomous drone navigation.
"""

import numpy as np
import open3d as o3d
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any, Optional


@dataclass
class ObstacleCluster:
    """
    Represents an extracted 3D spatial obstacle.
    """
    cluster_id: int
    centroid: np.ndarray
    min_bound: np.ndarray
    max_bound: np.ndarray
    dimensions: np.ndarray
    volume_m3: float
    point_count: int
    points: np.ndarray

    def distance_to_point(self, pt: np.ndarray) -> float:
        """Computes minimum Euclidean distance from a point to this obstacle's bounding box."""
        # Clamped distance to Axis-Aligned Bounding Box
        clamped = np.clip(pt, self.min_bound, self.max_bound)
        return float(np.linalg.norm(pt - clamped))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "cluster_id": self.cluster_id,
            "centroid": [round(float(x), 3) for x in self.centroid],
            "dimensions_xyz": [round(float(x), 3) for x in self.dimensions],
            "volume_m3": round(float(self.volume_m3), 4),
            "point_count": self.point_count,
            "min_bound": [round(float(x), 3) for x in self.min_bound],
            "max_bound": [round(float(x), 3) for x in self.max_bound]
        }


class ObstacleExtractor:
    """
    Extracts, clusters, and evaluates 3D obstacles from occupied point clouds / voxels.
    """

    def __init__(
        self,
        cluster_eps: float = 0.25,
        min_cluster_points: int = 25,
        drone_safety_radius_m: float = 0.40
    ):
        self.cluster_eps = float(cluster_eps)
        self.min_cluster_points = int(min_cluster_points)
        self.drone_safety_radius_m = float(drone_safety_radius_m)

    def extract_obstacles(self, points: np.ndarray) -> List[ObstacleCluster]:
        """
        Segments 3D points into distinct obstacle clusters using DBSCAN.
        """
        if len(points) < self.min_cluster_points:
            return []

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # DBSCAN Euclidean clustering
        labels = np.array(pcd.cluster_dbscan(
            eps=self.cluster_eps,
            min_points=self.min_cluster_points,
            print_progress=False
        ))

        max_label = labels.max()
        if max_label < 0:
            return []

        obstacles = []
        for c_id in range(max_label + 1):
            c_mask = (labels == c_id)
            c_pts = points[c_mask]

            min_b = np.min(c_pts, axis=0)
            max_b = np.max(c_pts, axis=0)
            dims = max_b - min_b
            centroid = (min_b + max_b) * 0.5
            vol = float(np.prod(dims))

            cluster = ObstacleCluster(
                cluster_id=c_id,
                centroid=centroid,
                min_bound=min_b,
                max_bound=max_b,
                dimensions=dims,
                volume_m3=vol,
                point_count=len(c_pts),
                points=c_pts
            )
            obstacles.append(cluster)

        # Sort obstacles by volume (largest first)
        obstacles.sort(key=lambda o: o.volume_m3, reverse=True)
        return obstacles

    def evaluate_drone_safety(
        self,
        drone_position: np.ndarray,
        obstacles: List[ObstacleCluster]
    ) -> Dict[str, Any]:
        """
        Assesses real-time collision safety for drone position against all extracted obstacles.
        """
        if not obstacles:
            return {
                "status": "SAFE_CLEAR",
                "min_distance_m": float("inf"),
                "nearest_obstacle_id": -1,
                "in_collision": False
            }

        min_dist = float("inf")
        nearest_id = -1

        for obs in obstacles:
            d = obs.distance_to_point(drone_position)
            if d < min_dist:
                min_dist = d
                nearest_id = obs.cluster_id

        in_collision = min_dist <= 0.0
        is_warning = min_dist < self.drone_safety_radius_m

        if in_collision:
            status = "COLLISION_CRITICAL"
        elif is_warning:
            status = "PROXIMITY_WARNING"
        else:
            status = "SAFE_CLEAR"

        return {
            "status": status,
            "min_distance_m": round(min_dist, 3),
            "safety_radius_m": self.drone_safety_radius_m,
            "nearest_obstacle_id": nearest_id,
            "in_collision": in_collision
        }
