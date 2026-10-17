#!/usr/bin/env python3
"""
3-State Volumetric Occupancy Grid Engine.
Implements rigorous tri-state volumetric representation:
  { FREE, OCCUPIED, UNKNOWN }
Uses line-of-sight raycasting from camera trajectory optical centers (p_WC)
to carve free space and classify unobserved volumes as UNKNOWN (preventing unsafe assumptions).
"""

import numpy as np
import open3d as o3d
from typing import Tuple, List, Optional, Dict, Any


class OccupancyGrid3D:
    """
    Volumetric 3-State Occupancy Grid:
      0: UNKNOWN  (never observed by camera field-of-view)
      1: FREE     (carved by clear line-of-sight visual ray)
      2: OCCUPIED (contains reconstructed 3D object surface)
    """

    STATE_UNKNOWN = 0
    STATE_FREE = 1
    STATE_OCCUPIED = 2

    def __init__(
        self,
        min_bound: np.ndarray = np.array([-4.5, -4.5, -0.5]),
        max_bound: np.ndarray = np.array([4.5, 4.5, 4.5]),
        voxel_size: float = 0.12
    ):
        self.min_bound = np.asarray(min_bound, dtype=np.float64)
        self.max_bound = np.asarray(max_bound, dtype=np.float64)
        self.voxel_size = float(voxel_size)

        self.dims = np.ceil((self.max_bound - self.min_bound) / self.voxel_size).astype(int)
        # 3D grid array initialized to UNKNOWN
        self.grid = np.zeros(self.dims, dtype=np.uint8)

    def world_to_grid(self, pt: np.ndarray) -> Optional[np.ndarray]:
        """Maps 3D World coordinate (meters) to 3D grid integer index."""
        idx = np.floor((pt - self.min_bound) / self.voxel_size).astype(int)
        if np.all(idx >= 0) and np.all(idx < self.dims):
            return idx
        return None

    def grid_to_world(self, idx: np.ndarray) -> np.ndarray:
        """Maps 3D grid integer index to voxel center in World coordinates (meters)."""
        return self.min_bound + (idx + 0.5) * self.voxel_size

    def insert_point_cloud_and_rays(
        self,
        surface_points: np.ndarray,
        camera_centers: List[np.ndarray],
        max_ray_stride: int = 5
    ):
        """
        Populates grid using line-of-sight raycasting:
          1. Sets surface points as OCCUPIED.
          2. Carves rays between camera centers and surface points as FREE.
          3. Unvisited space remains UNKNOWN.
        """
        if len(surface_points) == 0:
            return

        print(f"[OccupancyGrid] Integrating {len(surface_points)} surface points and {len(camera_centers)} viewpoints...")

        # 1. Mark Surface Points as OCCUPIED
        for pt in surface_points:
            g_idx = self.world_to_grid(pt)
            if g_idx is not None:
                self.grid[g_idx[0], g_idx[1], g_idx[2]] = self.STATE_OCCUPIED

        # 2. Line-of-sight raycasting (Carve FREE space)
        # To maintain high performance, we trace rays with discrete 3D sampling
        step = self.voxel_size * 0.75
        for cam_idx, cam_p in enumerate(camera_centers):
            # Sample subset of points visible from this camera center
            sub_pts = surface_points[::max_ray_stride]
            for pt in sub_pts:
                ray_vec = pt - cam_p
                dist = np.linalg.norm(ray_vec)
                if dist < 0.2:
                    continue

                dir_norm = ray_vec / dist
                # Trace from camera origin up to surface point - 1 voxel margin
                num_steps = int(max(1, (dist - self.voxel_size * 1.2) / step))
                for s in range(1, num_steps):
                    ray_p = cam_p + dir_norm * (s * step)
                    g_idx = self.world_to_grid(ray_p)
                    if g_idx is not None:
                        # Only carve if not already occupied
                        if self.grid[g_idx[0], g_idx[1], g_idx[2]] != self.STATE_OCCUPIED:
                            self.grid[g_idx[0], g_idx[1], g_idx[2]] = self.STATE_FREE

        stats = self.get_statistics()
        print(f"[OccupancyGrid] Classification complete: "
              f"Occupied: {stats['occupied_count']}, "
              f"Free: {stats['free_count']}, "
              f"Unknown: {stats['unknown_count']} voxels.")

    def get_state(self, x: float, y: float, z: float) -> int:
        """Returns state: 0 (UNKNOWN), 1 (FREE), 2 (OCCUPIED)."""
        idx = self.world_to_grid(np.array([x, y, z]))
        if idx is None:
            return self.STATE_UNKNOWN
        return int(self.grid[idx[0], idx[1], idx[2]])

    def is_sphere_safe(self, center: np.ndarray, radius: float = 0.3) -> Tuple[bool, str]:
        """
        Checks if a spherical collision bubble around position is safe for drone flight.
        Fails if ANY voxel within radius is OCCUPIED or UNKNOWN!
        """
        r_vox = int(np.ceil(radius / self.voxel_size))
        c_idx = self.world_to_grid(center)
        if c_idx is None:
            return False, "OUT_OF_BOUNDS"

        for dx in range(-r_vox, r_vox + 1):
            for dy in range(-r_vox, r_vox + 1):
                for dz in range(-r_vox, r_vox + 1):
                    dist = np.sqrt(dx**2 + dy**2 + dz**2) * self.voxel_size
                    if dist <= radius:
                        idx = c_idx + np.array([dx, dy, dz])
                        if np.all(idx >= 0) and np.all(idx < self.dims):
                            state = self.grid[idx[0], idx[1], idx[2]]
                            if state == self.STATE_OCCUPIED:
                                return False, "OCCUPIED_COLLISION"
                            elif state == self.STATE_UNKNOWN:
                                return False, "UNKNOWN_RISK"
        return True, "SAFE_FREE"

    def get_occupied_voxel_centers(self) -> np.ndarray:
        """Returns (N, 3) coordinates of all occupied voxel centers."""
        indices = np.argwhere(self.grid == self.STATE_OCCUPIED)
        if len(indices) == 0:
            return np.empty((0, 3))
        return self.min_bound + (indices + 0.5) * self.voxel_size

    def get_free_voxel_centers(self, max_samples: int = 15000) -> np.ndarray:
        """Returns sample coordinates of free space voxels for visualization."""
        indices = np.argwhere(self.grid == self.STATE_FREE)
        if len(indices) == 0:
            return np.empty((0, 3))
        if len(indices) > max_samples:
            indices = indices[np.random.choice(len(indices), max_samples, replace=False)]
        return self.min_bound + (indices + 0.5) * self.voxel_size

    def get_statistics(self) -> Dict[str, Any]:
        """Computes volumetric occupancy distribution."""
        total = int(self.grid.size)
        occ = int(np.sum(self.grid == self.STATE_OCCUPIED))
        free = int(np.sum(self.grid == self.STATE_FREE))
        unk = int(np.sum(self.grid == self.STATE_UNKNOWN))
        return {
            "total_voxels": total,
            "occupied_count": occ,
            "free_count": free,
            "unknown_count": unk,
            "occupied_volume_m3": round(occ * (self.voxel_size ** 3), 3),
            "free_volume_m3": round(free * (self.voxel_size ** 3), 3),
            "unknown_volume_m3": round(unk * (self.voxel_size ** 3), 3)
        }

    def export_pointcloud(self, output_path: str):
        """Exports occupied voxels as a colored Open3D point cloud."""
        occ_pts = self.get_occupied_voxel_centers()
        if len(occ_pts) == 0:
            return
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(occ_pts)
        pcd.paint_uniform_color([0.9, 0.2, 0.2]) # Crimson Red for obstacles
        o3d.io.write_point_cloud(output_path, pcd)
        print(f"[OccupancyGrid] Exported {len(occ_pts)} occupied voxels to {output_path}")
