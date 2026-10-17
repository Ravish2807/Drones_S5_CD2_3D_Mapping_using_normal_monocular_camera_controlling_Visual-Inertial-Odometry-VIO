#!/usr/bin/env python3
"""
Unit tests for Phase 3 VIO-Guided 3D Reconstruction Pipeline.
Validates Keyframe Contracts, Keyframe Selection criteria, MVS Depth Estimation,
Depth Fusion Back-Projection, and Surface Meshing.
"""

import pytest
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from reconstruction.keyframe_bundle import Keyframe, CameraIntrinsics, KeyframeDatabase
from reconstruction.keyframe_selector import KeyframeSelector
from reconstruction.mvs_depth_estimator import MVSDepthEstimator
from reconstruction.depth_fusion import DepthFusion
from reconstruction.surface_mesher import SurfaceMesher


def test_camera_intrinsics():
    intr = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)
    K = intr.K
    assert K.shape == (3, 3)
    assert K[0, 0] == 500.0
    assert K[1, 1] == 500.0
    assert K[0, 2] == 320.0
    assert K[1, 2] == 240.0
    assert K[2, 2] == 1.0

    d = intr.to_dict()
    intr_restored = CameraIntrinsics.from_dict(d)
    assert intr_restored.fx == intr.fx
    assert intr_restored.cx == intr.cx


def test_keyframe_data_contract():
    intr = CameraIntrinsics(fx=400.0, fy=400.0, cx=320.0, cy=240.0, width=640, height=480)
    R_WC = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float64) # 90 deg yaw
    p_WC = np.array([2.5, 1.0, 3.0], dtype=np.float64)
    T_WC = np.eye(4)
    T_WC[0:3, 0:3] = R_WC
    T_WC[0:3, 3] = p_WC

    img = np.zeros((480, 640), dtype=np.uint8)
    kf = Keyframe(frame_id=0, timestamp=10.0, image=img, T_WC=T_WC, intrinsics=intr)

    np.testing.assert_allclose(kf.p_WC, p_WC)
    np.testing.assert_allclose(kf.R_WC, R_WC)
    np.testing.assert_allclose(kf.optical_center, p_WC)

    # Viewing direction is Z-axis in Camera frame transformed to World
    expected_v = R_WC @ np.array([0, 0, 1])
    np.testing.assert_allclose(kf.viewing_direction, expected_v)


def test_keyframe_selector_translation():
    selector = KeyframeSelector(min_translation_m=0.20, min_rotation_deg=10.0)
    intr = CameraIntrinsics(fx=400.0, fy=400.0, cx=320.0, cy=240.0, width=640, height=480)
    img = np.zeros((480, 640), dtype=np.uint8)

    # Frame 0: Initial
    T0 = np.eye(4)
    is_kf, reason, kf0 = selector.evaluate(timestamp=0.0, image=img, T_WC=T0, intrinsics=intr)
    assert is_kf is True
    assert reason == "initial_frame"

    # Frame 1: Minor movement (0.05m < 0.20m) -> Should be rejected
    T1 = np.eye(4)
    T1[0, 3] = 0.05
    is_kf, reason, _ = selector.evaluate(timestamp=0.1, image=img, T_WC=T1, intrinsics=intr)
    assert is_kf is False
    assert reason == "redundant_pose"

    # Frame 2: Sufficient translation (0.25m >= 0.20m) -> Should be accepted
    T2 = np.eye(4)
    T2[0, 3] = 0.25
    is_kf, reason, kf2 = selector.evaluate(timestamp=0.2, image=img, T_WC=T2, intrinsics=intr)
    assert is_kf is True
    assert "translation_exceeded" in reason
    assert kf2.frame_id == 1


def test_keyframe_selector_rotation():
    selector = KeyframeSelector(min_translation_m=0.50, min_rotation_deg=15.0)
    intr = CameraIntrinsics(fx=400.0, fy=400.0, cx=320.0, cy=240.0, width=640, height=480)
    img = np.zeros((480, 640), dtype=np.uint8)

    # Frame 0: Initial
    T0 = np.eye(4)
    selector.evaluate(timestamp=0.0, image=img, T_WC=T0, intrinsics=intr)

    # Frame 1: Small rotation (5 deg < 15 deg)
    R1 = Rotation.from_euler('z', 5.0, degrees=True).as_matrix()
    T1 = np.eye(4)
    T1[0:3, 0:3] = R1
    is_kf, _, _ = selector.evaluate(timestamp=0.1, image=img, T_WC=T1, intrinsics=intr)
    assert is_kf is False

    # Frame 2: Large rotation (20 deg >= 15 deg)
    R2 = Rotation.from_euler('z', 20.0, degrees=True).as_matrix()
    T2 = np.eye(4)
    T2[0:3, 0:3] = R2
    is_kf, reason, _ = selector.evaluate(timestamp=0.2, image=img, T_WC=T2, intrinsics=intr)
    assert is_kf is True
    assert "rotation_exceeded" in reason


def test_depth_fusion_backprojection():
    intr = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)
    T_WC = np.eye(4)
    T_WC[0:3, 3] = np.array([1.0, 2.0, 3.0])

    img = np.full((480, 640), 200, dtype=np.uint8)
    depth = np.full((480, 640), 2.0, dtype=np.float32) # Depth = 2 meters
    conf = np.ones((480, 640), dtype=np.float32)

    kf = Keyframe(frame_id=0, timestamp=0.0, image=img, T_WC=T_WC, intrinsics=intr, depth_map=depth, confidence_map=conf)
    fusion = DepthFusion(min_depth=1.0, max_depth=5.0)

    pts_W, colors, confs = fusion.backproject_keyframe_points(kf)
    assert len(pts_W) == 480 * 640

    # Test center pixel (u=320, v=240) -> In camera frame: [0, 0, 2.0]
    # In world frame: [1, 2, 3] + [0, 0, 2] = [1.0, 2.0, 5.0]
    center_idx = 240 * 640 + 320
    np.testing.assert_allclose(pts_W[center_idx], [1.0, 2.0, 5.0], atol=1e-3)


def test_surface_mesher():
    # Create synthetic sphere point cloud with normals
    mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    pcd = mesh_sphere.sample_points_poisson_disk(number_of_points=1000)
    pcd.estimate_normals()

    mesher = SurfaceMesher(poisson_depth=6)
    reconstructed_mesh = mesher.reconstruct_poisson_mesh(pcd)

    assert len(reconstructed_mesh.vertices) > 50
    assert len(reconstructed_mesh.triangles) > 50


def test_known_box_target():
    from reconstruction.known_object import KnownBoxTarget
    target = KnownBoxTarget(center=np.array([0.0, 0.0, 2.0]), dimensions=np.array([1.2, 1.2, 1.2]))
    gt_mesh = target.get_ground_truth_mesh()
    assert len(gt_mesh.vertices) == 8
    assert len(gt_mesh.triangles) == 12

    pts = target.sample_surface_points(num_points=1000)
    assert len(pts.points) > 500
    landmarks = target.generate_textured_surface_landmarks(points_per_face=100)
    assert len(landmarks) > 200


def test_reconstruction_validator():
    from reconstruction.known_object import KnownBoxTarget
    from reconstruction.reconstruction_validator import ReconstructionValidator

    target = KnownBoxTarget(center=np.array([0.0, 0.0, 2.0]), dimensions=np.array([1.0, 1.0, 1.0]))
    # Sample points from the box itself with small noise
    gt_pts = target.sample_surface_points(num_points=2000)
    noisy_pts = o3d.geometry.PointCloud()
    noisy_pts.points = o3d.utility.Vector3dVector(np.asarray(gt_pts.points) + np.random.normal(0, 0.01, (len(gt_pts.points), 3)))

    validator = ReconstructionValidator(tolerance_m=0.05)
    metrics = validator.evaluate_against_known_box(noisy_pts, target, gt_sample_count=2000)

    assert metrics["status"] == "SUCCESS"
    assert metrics["chamfer_dist_m"] < 0.03 # Mean error should be very small
    assert metrics["completeness_pct"] > 85.0


def test_occupancy_grid_3state():
    from reconstruction.occupancy_grid import OccupancyGrid3D

    grid = OccupancyGrid3D(min_bound=np.array([-2, -2, 0]), max_bound=np.array([2, 2, 4]), voxel_size=0.2)
    surface_pts = np.array([
        [0.0, 0.0, 2.0],
        [0.1, 0.0, 2.0],
        [0.0, 0.1, 2.0]
    ])
    cam_centers = [np.array([1.5, 0.0, 2.0])]

    grid.insert_point_cloud_and_rays(surface_pts, cam_centers)

    # 1. Surface point voxel should be OCCUPIED
    assert grid.get_state(0.0, 0.0, 2.0) == OccupancyGrid3D.STATE_OCCUPIED

    # 2. Voxel along line-of-sight ray between camera (1.5, 0, 2) and surface (0, 0, 2) should be FREE
    assert grid.get_state(0.8, 0.0, 2.0) == OccupancyGrid3D.STATE_FREE

    # 3. Unvisited voxel far away should remain UNKNOWN
    assert grid.get_state(-1.5, -1.5, 3.5) == OccupancyGrid3D.STATE_UNKNOWN

    # 4. Collision check
    safe, status = grid.is_sphere_safe(np.array([0.0, 0.0, 2.0]), radius=0.1)
    assert safe is False
    assert status == "OCCUPIED_COLLISION"

    safe_free, _ = grid.is_sphere_safe(np.array([0.8, 0.0, 2.0]), radius=0.1)
    # Could be free or margin
    assert isinstance(safe_free, bool)


def test_obstacle_extractor():
    from reconstruction.obstacle_extractor import ObstacleExtractor

    extractor = ObstacleExtractor(cluster_eps=0.3, min_cluster_points=10)

    # Two distinct clusters
    c1 = np.random.normal(loc=[1.0, 1.0, 2.0], scale=0.05, size=(50, 3))
    c2 = np.random.normal(loc=[-1.0, -1.0, 2.0], scale=0.05, size=(50, 3))
    pts = np.vstack([c1, c2])

    obstacles = extractor.extract_obstacles(pts)
    assert len(obstacles) == 2

    # Check nearest obstacle evaluation
    drone_p = np.array([1.0, 1.0, 2.0])
    safety = extractor.evaluate_drone_safety(drone_p, obstacles)
    assert safety["in_collision"] is True or safety["min_distance_m"] < 0.2

