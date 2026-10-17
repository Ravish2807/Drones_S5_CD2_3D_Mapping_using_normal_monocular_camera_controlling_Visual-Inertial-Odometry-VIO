#!/usr/bin/env python3
"""
Known Ground-Truth Target Object Definition for Reconstruction Validation.
Generates an analytically exact 3D Box with known metric dimensions, surface points,
analytical triangle meshes, and textured visual surface landmarks for controlled MVS testing.
"""

import numpy as np
import open3d as o3d
from typing import Tuple, Dict, Any, List


class KnownBoxTarget:
    """
    Analytically defined 3D rectangular box target object.
    Default: 1.2m x 1.2m x 1.2m cube centered at [0.0, 0.0, 2.0] m in World coordinates.
    """

    def __init__(
        self,
        center: np.ndarray = np.array([0.0, 0.0, 2.0]),
        dimensions: np.ndarray = np.array([1.2, 1.2, 1.2])
    ):
        self.center = np.asarray(center, dtype=np.float64)
        self.dimensions = np.asarray(dimensions, dtype=np.float64)
        self.half_extents = self.dimensions * 0.5

        self.min_bound = self.center - self.half_extents
        self.max_bound = self.center + self.half_extents

    def get_ground_truth_mesh(self) -> o3d.geometry.TriangleMesh:
        """Constructs the analytical ground-truth TriangleMesh for the box."""
        box = o3d.geometry.TriangleMesh.create_box(
            width=self.dimensions[0],
            height=self.dimensions[1],
            depth=self.dimensions[2]
        )
        # Shift from [0, 0, 0] base corner to desired center
        box.translate(self.min_bound)
        box.compute_vertex_normals()
        box.paint_uniform_color([0.3, 0.5, 0.9]) # Steel blue
        return box

    def sample_surface_points(self, num_points: int = 5000) -> o3d.geometry.PointCloud:
        """Uniformly samples points on all 6 faces of the ground truth box."""
        mesh = self.get_ground_truth_mesh()
        pcd = mesh.sample_points_poisson_disk(number_of_points=num_points)
        pcd.estimate_normals()
        return pcd

    def generate_textured_surface_landmarks(self, points_per_face: int = 150) -> np.ndarray:
        """
        Generates visually trackable grid landmarks on the 6 faces of the box.
        Used by the synthetic camera simulator to project optical features.
        """
        landmarks = []
        w, d, h = self.half_extents
        cx, cy, cz = self.center

        # Face definitions: (normal_axis, offset, axis1, axis2, range1, range2)
        faces = [
            # Front (+X)
            (np.array([cx + w, 0, 0]), 1, 2, np.linspace(cy - d, cy + d, int(np.sqrt(points_per_face))), np.linspace(cz - h, cz + h, int(np.sqrt(points_per_face)))),
            # Back (-X)
            (np.array([cx - w, 0, 0]), 1, 2, np.linspace(cy - d, cy + d, int(np.sqrt(points_per_face))), np.linspace(cz - h, cz + h, int(np.sqrt(points_per_face)))),
            # Right (+Y)
            (np.array([0, cy + d, 0]), 0, 2, np.linspace(cx - w, cx + w, int(np.sqrt(points_per_face))), np.linspace(cz - h, cz + h, int(np.sqrt(points_per_face)))),
            # Left (-Y)
            (np.array([0, cy - d, 0]), 0, 2, np.linspace(cx - w, cx + w, int(np.sqrt(points_per_face))), np.linspace(cz - h, cz + h, int(np.sqrt(points_per_face)))),
            # Top (+Z)
            (np.array([0, 0, cz + h]), 0, 1, np.linspace(cx - w, cx + w, int(np.sqrt(points_per_face))), np.linspace(cy - d, cy + d, int(np.sqrt(points_per_face)))),
            # Bottom (-Z)
            (np.array([0, 0, cz - h]), 0, 1, np.linspace(cx - w, cx + w, int(np.sqrt(points_per_face))), np.linspace(cy - d, cy + d, int(np.sqrt(points_per_face))))
        ]

        for base_pt, ax1, ax2, r1, r2 in faces:
            for v1 in r1:
                for v2 in r2:
                    pt = base_pt.copy()
                    pt[ax1] = v1
                    pt[ax2] = v2
                    landmarks.append(pt)

        return np.array(landmarks, dtype=np.float64)
