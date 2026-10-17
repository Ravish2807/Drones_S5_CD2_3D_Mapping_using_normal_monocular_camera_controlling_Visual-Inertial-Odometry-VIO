#!/usr/bin/env python3
"""
Reconstruction Geometric Validation Engine.
Computes quantitative accuracy metrics comparing a reconstructed 3D point cloud / mesh
against an analytical Ground-Truth object:
  - Symmetric Chamfer Distance (meters)
  - Hausdorff Distance (meters)
  - Surface Completeness (% of GT covered within threshold)
  - Reconstruction Precision (% of reconstructed points near GT)
  - Reconstruction Density (points / m^2)
  - Bounding Box Dimensions vs Analytical Truth
"""

import numpy as np
import open3d as o3d
from typing import Dict, Any, Tuple
from .known_object import KnownBoxTarget


class ReconstructionValidator:
    """
    Validates reconstructed 3D geometry against analytical ground-truth objects.
    """

    def __init__(self, tolerance_m: float = 0.08):
        self.tolerance_m = float(tolerance_m)

    def evaluate_against_known_box(
        self,
        reconstructed_pcd: o3d.geometry.PointCloud,
        target_box: KnownBoxTarget,
        gt_sample_count: int = 5000
    ) -> Dict[str, Any]:
        """
        Computes full set of SLAM / MVS geometric evaluation metrics.
        """
        rec_pts = np.asarray(reconstructed_pcd.points)
        if len(rec_pts) == 0:
            return {
                "status": "FAILED_EMPTY",
                "point_count": 0,
                "chamfer_dist_m": float("inf"),
                "completeness_pct": 0.0
            }

        # 1. Sample analytical ground-truth points
        gt_pcd = target_box.sample_surface_points(num_points=gt_sample_count)
        gt_pts = np.asarray(gt_pcd.points)

        # 2. Build KD-Trees for bidirectional nearest neighbor querying
        rec_kdtree = o3d.geometry.KDTreeFlann(reconstructed_pcd)
        gt_kdtree = o3d.geometry.KDTreeFlann(gt_pcd)

        # A. Rec -> GT distances (Accuracy / Precision)
        dist_rec_to_gt = []
        for p in rec_pts:
            [_, idx, dist_sq] = gt_kdtree.search_knn_vector_3d(p, 1)
            dist_rec_to_gt.append(np.sqrt(dist_sq[0]))
        dist_rec_to_gt = np.array(dist_rec_to_gt)

        # B. GT -> Rec distances (Completeness / Coverage)
        dist_gt_to_rec = []
        for q in gt_pts:
            [_, idx, dist_sq] = rec_kdtree.search_knn_vector_3d(q, 1)
            dist_gt_to_rec.append(np.sqrt(dist_sq[0]))
        dist_gt_to_rec = np.array(dist_gt_to_rec)

        # 3. Geometric Metric Computations
        mean_rec_to_gt = float(np.mean(dist_rec_to_gt))
        mean_gt_to_rec = float(np.mean(dist_gt_to_rec))
        chamfer_dist = (mean_rec_to_gt + mean_gt_to_rec) * 0.5
        hausdorff_dist = float(max(np.max(dist_rec_to_gt), np.max(dist_gt_to_rec)))

        completeness_pct = float(np.mean(dist_gt_to_rec <= self.tolerance_m) * 100.0)
        precision_pct = float(np.mean(dist_rec_to_gt <= self.tolerance_m) * 100.0)

        # 4. Dimensional comparison
        rec_bbox = reconstructed_pcd.get_axis_aligned_bounding_box()
        rec_dims = rec_bbox.get_extent()
        gt_dims = target_box.dimensions

        dim_errors = np.abs(rec_dims - gt_dims)

        # Surface area of box: 2*(w*d + w*h + d*h)
        w, d, h = gt_dims
        box_surface_area = 2.0 * (w * d + w * h + d * h)
        density = len(rec_pts) / box_surface_area

        return {
            "status": "SUCCESS",
            "point_count": len(rec_pts),
            "chamfer_dist_m": round(chamfer_dist, 4),
            "mean_accuracy_m": round(mean_rec_to_gt, 4),
            "mean_completeness_dist_m": round(mean_gt_to_rec, 4),
            "hausdorff_dist_m": round(hausdorff_dist, 4),
            "completeness_pct": round(completeness_pct, 2),
            "precision_pct": round(precision_pct, 2),
            "tolerance_threshold_m": self.tolerance_m,
            "density_points_per_m2": round(density, 1),
            "ground_truth_dimensions_m": gt_dims.tolist(),
            "reconstructed_dimensions_m": [round(float(x), 3) for x in rec_dims],
            "dimensional_errors_m": [round(float(x), 3) for x in dim_errors]
        }
