#!/usr/bin/env python3
"""
Surface Mesh Reconstruction Engine.
Converts dense 3D point clouds with surface normals into continuous, watertight 3D meshes
using Screened Poisson Surface Reconstruction and the Ball-Pivoting Algorithm (BPA).
"""

import os
import numpy as np
import open3d as o3d
from typing import Optional, Tuple


class SurfaceMesher:
    """
    Transforms dense point clouds into continuous 3D surface meshes
    and generates obstacle occupancy voxel grids.
    """

    def __init__(
        self,
        poisson_depth: int = 9,
        density_quantile_trim: float = 0.05,
        bpa_ball_radii: Optional[list] = None
    ):
        self.poisson_depth = int(poisson_depth)
        self.density_quantile_trim = float(density_quantile_trim)
        self.bpa_ball_radii = bpa_ball_radii or [0.03, 0.06, 0.12]

    def reconstruct_poisson_mesh(
        self,
        pcd: o3d.geometry.PointCloud
    ) -> o3d.geometry.TriangleMesh:
        """
        Screened Poisson Surface Reconstruction.
        Generates watertight, smooth continuous 3D surface meshes from oriented point clouds.
        """
        print(f"[SurfaceMesher] Running Screened Poisson Reconstruction (depth={self.poisson_depth})...")

        # Ensure normals are estimated
        if not pcd.has_normals():
            print("[SurfaceMesher] Estimating normals before Poisson reconstruction...")
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            pcd.orient_normals_consistent_tangent_plane(k=15)

        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=self.poisson_depth, width=0, scale=1.1, linear_fit=False
        )

        # Trim low density artifact vertices
        densities_np = np.asarray(densities)
        if len(densities_np) > 0:
            density_threshold = np.quantile(densities_np, self.density_quantile_trim)
            vertices_to_remove = densities_np < density_threshold
            mesh.remove_vertices_by_mask(vertices_to_remove)

        # Clean up mesh topology
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_duplicated_triangles()
        mesh.remove_non_manifold_edges()
        mesh.compute_vertex_normals()

        # Transfer colors from point cloud if points have colors
        if pcd.has_colors() and len(mesh.vertices) > 0:
            pcd_tree = o3d.geometry.KDTreeFlann(pcd)
            vertex_colors = []
            pcd_colors = np.asarray(pcd.colors)
            for v in np.asarray(mesh.vertices):
                [_, idx, _] = pcd_tree.search_knn_vector_3d(v, 1)
                vertex_colors.append(pcd_colors[idx[0]])
            mesh.vertex_colors = o3d.utility.Vector3dVector(np.array(vertex_colors))

        print(f"[SurfaceMesher] Poisson mesh created: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles.")
        return mesh

    def reconstruct_bpa_mesh(
        self,
        pcd: o3d.geometry.PointCloud
    ) -> o3d.geometry.TriangleMesh:
        """
        Ball-Pivoting Algorithm (BPA) reconstruction.
        Preserves sharp object boundaries and raw point geometry.
        """
        print(f"[SurfaceMesher] Running Ball-Pivoting Algorithm (radii={self.bpa_ball_radii})...")
        if not pcd.has_normals():
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        radii = o3d.utility.DoubleVector(self.bpa_ball_radii)
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, radii)
        mesh.compute_vertex_normals()
        print(f"[SurfaceMesher] BPA mesh created: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles.")
        return mesh

    def export_mesh(self, mesh: o3d.geometry.TriangleMesh, output_path: str):
        """Saves 3D surface mesh to disk (.ply or .obj)."""
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        o3d.io.write_triangle_mesh(output_path, mesh)
        print(f"[SurfaceMesher] Saved 3D surface mesh to: {output_path}")

    def export_voxel_grid(self, pcd: o3d.geometry.PointCloud, output_path: str, voxel_size: float = 0.10):
        """Generates and saves 3D occupancy voxel grid map for drone obstacle avoidance."""
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        voxels = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)
        o3d.io.write_voxel_grid(output_path, voxels)
        print(f"[SurfaceMesher] Saved 3D obstacle voxel grid to: {output_path}")
