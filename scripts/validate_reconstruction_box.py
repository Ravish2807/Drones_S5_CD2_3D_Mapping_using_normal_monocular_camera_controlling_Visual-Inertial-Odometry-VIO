#!/usr/bin/env python3
"""
Phase 3 & 4A Controlled Validation Experiment: Known 3D Box Target.
Validates the reconstruction pipeline against an analytical ground-truth object:
  1. Places a 1.2m x 1.2m x 1.2m textured box at scene center [0, 0, 2.0].
  2. Runs VIO camera tracking T_WC(t) during an inspection orbit.
  3. Evaluates Keyframe selection around the box.
  4. Runs MVS Depth Estimation & Depth Fusion to reconstruct the box.
  5. Computes Chamfer Distance, Completeness, and Dimensional Accuracy.
  6. Constructs 3-State Occupancy Grid (FREE, OCCUPIED, UNKNOWN).
  7. Extracts Obstacle Clusters & Bounding Volumes for drone collision avoidance.
"""

import os
import sys
import yaml
import json
import time
import argparse
import numpy as np
import open3d as o3d
import cv2

# Ensure workspace root in path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from datasets.synthetic.synthetic_generator import SyntheticDatasetGenerator
from interfaces.vio_interface import VIOInterface
from trajectory.camera_pose import CameraExtrinsics
from reconstruction.keyframe_bundle import Keyframe, CameraIntrinsics, KeyframeDatabase
from reconstruction.keyframe_selector import KeyframeSelector
from reconstruction.mvs_depth_estimator import MVSDepthEstimator
from reconstruction.depth_fusion import DepthFusion
from reconstruction.surface_mesher import SurfaceMesher
from reconstruction.known_object import KnownBoxTarget
from reconstruction.reconstruction_validator import ReconstructionValidator
from reconstruction.occupancy_grid import OccupancyGrid3D
from reconstruction.obstacle_extractor import ObstacleExtractor


def main():
    parser = argparse.ArgumentParser(description="Controlled Known Box Reconstruction & Obstacle Extraction Experiment")
    parser.add_argument("--config", type=str, default="config/synthetic.yaml", help="Path to config YAML")
    parser.add_argument("--out_dir", type=str, default="results", help="Directory to save artifacts")
    parser.add_argument("--visualize", action="store_true", help="Launch interactive 3D viewer upon completion")
    args = parser.parse_args()

    print("=" * 80)
    print("  PHASE 3 & 4A EXPERIMENT: CONTROLLED KNOWN BOX RECONSTRUCTION & OBSTACLE MAPPING")
    print("=" * 80)

    # 1. Define Known Ground-Truth Object (1.2m cube centered at [0, 0, 2.0])
    target_box = KnownBoxTarget(center=np.array([0.0, 0.0, 2.0]), dimensions=np.array([1.2, 1.2, 1.2]))
    print(f"\n[1/6] Ground Truth Object Defined:")
    print(f"      * Geometry:   Rectangular Box")
    print(f"      * Centroid:   {target_box.center.tolist()} m")
    print(f"      * Dimensions: {target_box.dimensions.tolist()} m (Volume: {np.prod(target_box.dimensions):.3f} m^3)")

    # 2. Setup VIO Simulation with Camera Orbiting Around the Box
    with open(args.config, "r") as f:
        config = yaml.safe_load(f)

    out_dir = args.out_dir
    pcd_dir = os.path.join(out_dir, "pointclouds")
    kf_dir = os.path.join(out_dir, "keyframes")
    metrics_dir = os.path.join(out_dir, "metrics")
    os.makedirs(pcd_dir, exist_ok=True)
    os.makedirs(kf_dir, exist_ok=True)
    os.makedirs(metrics_dir, exist_ok=True)

    cam_cfg = config.get("camera", {})
    intr_cfg = cam_cfg.get("intrinsics", {})
    intrinsics = CameraIntrinsics(
        fx=float(intr_cfg.get("fx", 400.0)),
        fy=float(intr_cfg.get("fy", 400.0)),
        cx=float(intr_cfg.get("cx", 320.0)),
        cy=float(intr_cfg.get("cy", 240.0)),
        width=int(cam_cfg.get("width", 640)),
        height=int(cam_cfg.get("height", 480))
    )
    extrinsics = CameraExtrinsics.from_config(config)

    generator = SyntheticDatasetGenerator(config)
    # Generate structured visual surface texture landmarks on the box faces
    box_landmarks = target_box.generate_textured_surface_landmarks(points_per_face=120)
    generator.landmarks_W = box_landmarks

    print(f"\n[2/6] Simulating Drone Orbit Inspection:")
    print(f"      * Box Surface Visual Landmarks: {len(box_landmarks)}")
    dataset = generator.generate_dataset(preset="inspect_orbit")
    imu_stream = dataset["imu"]
    cam_stream = dataset["camera"]
    gt_stream = dataset["ground_truth"]

    # 3. Execute VIO & Keyframe Selection
    print(f"\n[3/6] Running MSCKF VIO & Extracting Multi-View Keyframes...")
    vio = VIOInterface(config)
    gt0 = gt_stream[0]
    vio.estimator.p_WB = gt0["position"].copy()
    vio.estimator.v_WB = gt0["velocity"].copy()
    vio.estimator.R_WB = gt0["R_WB"].copy()
    vio.estimator.is_initialized = True

    selector = KeyframeSelector(
        min_translation_m=0.45,
        min_rotation_deg=14.0,
        min_feature_overlap=0.55
    )
    db = KeyframeDatabase()

    imu_idx = 0
    cam_idx = 0
    n_imu = len(imu_stream)
    n_cam = len(cam_stream)

    t0_vio = time.time()
    while imu_idx < n_imu or cam_idx < n_cam:
        imu_t = imu_stream[imu_idx]["timestamp"] if imu_idx < n_imu else float("inf")
        cam_t = cam_stream[cam_idx]["timestamp"] if cam_idx < n_cam else float("inf")

        if imu_t <= cam_t and imu_idx < n_imu:
            sample = imu_stream[imu_idx]
            vio.push_imu(sample["timestamp"], *sample["linear_accel"], *sample["angular_vel"])
            imu_idx += 1
        elif cam_idx < n_cam:
            frame = cam_stream[cam_idx]
            timestamp = frame["timestamp"]
            img = frame["image"]

            state = vio.push_camera_frame(timestamp, img)
            if state is None:
                state = vio.get_current_state()

            if state is not None:
                p_WB = state.position
                R_WB = state.orientation_R

                # Transform to Camera Optical Pose: T_WC = T_WB * T_BC
                p_WC = p_WB + R_WB @ extrinsics.p_BC
                R_WC = R_WB @ extrinsics.R_BC
                T_WC = np.eye(4)
                T_WC[0:3, 0:3] = R_WC
                T_WC[0:3, 3] = p_WC

                feature_ids = list(vio.estimator.tracker.prev_ids) if vio.estimator.tracker.prev_ids is not None else None
                is_kf, reason, kf = selector.evaluate(
                    timestamp=timestamp,
                    image=img,
                    T_WC=T_WC,
                    intrinsics=intrinsics,
                    feature_ids=feature_ids
                )
                if is_kf and kf is not None:
                    db.add(kf)

            cam_idx += 1

    print(f"      * VIO Time: {time.time() - t0_vio:.2f}s")
    print(f"      * Selected {len(db)} keyframes around the target box ({len(db)/selector.total_frames_evaluated*100:.1f}% reduction).")

    # 4. Multi-View Stereo Depth Estimation & Depth Fusion
    print(f"\n[4/6] Executing Multi-View Stereo & Depth Map Fusion...")
    depth_estimator = MVSDepthEstimator(
        min_depth=1.5,
        max_depth=5.5,
        num_depth_planes=48,
        patch_size=5,
        method="plane_sweep"
    )

    t0_mvs = time.time()
    for i in range(len(db)):
        depth_estimator.process_keyframe(i, db.keyframes)

    fusion = DepthFusion(voxel_size_m=0.05, min_depth=1.5, max_depth=5.5, confidence_thresh=0.08)
    reconstructed_pcd = fusion.fuse_point_clouds(db.keyframes)

    box_pcd_path = os.path.join(pcd_dir, "box_reconstructed.ply")
    fusion.export_point_cloud(reconstructed_pcd, box_pcd_path)

    mesher = SurfaceMesher(poisson_depth=7)
    box_mesh = mesher.reconstruct_poisson_mesh(reconstructed_pcd)
    box_mesh_path = os.path.join(pcd_dir, "box_reconstructed_mesh.ply")
    mesher.export_mesh(box_mesh, box_mesh_path)
    print(f"      * MVS & Fusion Time: {time.time() - t0_mvs:.2f}s")

    # 5. Quantitative Geometric Validation
    print(f"\n[5/6] Computing Geometric Error Metrics vs Ground Truth Box...")
    validator = ReconstructionValidator(tolerance_m=0.08)
    geom_metrics = validator.evaluate_against_known_box(reconstructed_pcd, target_box)

    print("      --------------------------------------------------")
    print(f"      * Point Count:           {geom_metrics['point_count']}")
    print(f"      * Chamfer Distance:      {geom_metrics['chamfer_dist_m']:.4f} m ({geom_metrics['chamfer_dist_m']*100:.2f} cm)")
    print(f"      * Surface Completeness:  {geom_metrics['completeness_pct']:.1f}% (within 8cm)")
    print(f"      * Precision:             {geom_metrics['precision_pct']:.1f}%")
    print(f"      * Reconstruction Density: {geom_metrics['density_points_per_m2']} pts/m^2")
    print(f"      * GT Dimensions:         {geom_metrics['ground_truth_dimensions_m']} m")
    print(f"      * Reconstructed Dims:    {geom_metrics['reconstructed_dimensions_m']} m")
    print(f"      * Dimensional Error:     {geom_metrics['dimensional_errors_m']} m")
    print("      --------------------------------------------------")

    # 6. Phase 4A: 3-State Occupancy Grid & Obstacle Extraction
    print(f"\n[6/6] Phase 4A: 3-State Occupancy Mapping & Obstacle Extraction...")
    occ_grid = OccupancyGrid3D(
        min_bound=np.array([-4.0, -4.0, 0.0]),
        max_bound=np.array([4.0, 4.0, 4.0]),
        voxel_size=0.12
    )

    cam_centers = [kf.p_WC for kf in db.keyframes]
    occ_grid.insert_point_cloud_and_rays(
        surface_points=np.asarray(reconstructed_pcd.points),
        camera_centers=cam_centers
    )

    occ_stats = occ_grid.get_statistics()
    occ_pcd_path = os.path.join(pcd_dir, "occupancy_occupied.ply")
    occ_grid.export_pointcloud(occ_pcd_path)

    # Obstacle Extraction via DBSCAN Clustering
    extractor = ObstacleExtractor(cluster_eps=0.35, min_cluster_points=30, drone_safety_radius_m=0.45)
    obstacles = extractor.extract_obstacles(np.asarray(reconstructed_pcd.points))
    print(f"      * Extracted {len(obstacles)} 3D Obstacle Entity/Entities.")

    for obs in obstacles:
        print(f"        -> Obstacle #{obs.cluster_id}: Centroid={obs.centroid.tolist()}, Dims={obs.dimensions.tolist()}, Vol={obs.volume_m3:.3f} m^3")

    # Assess Drone Collision Safety at final pose
    final_drone_pos = gt_stream[-1]["position"]
    safety_eval = extractor.evaluate_drone_safety(final_drone_pos, obstacles)
    print(f"      * Final Drone Position: {final_drone_pos.tolist()}")
    print(f"      * Safety Status:        {safety_eval['status']} (Dist to obstacle: {safety_eval['min_distance_m']:.2f} m)")

    # Compile Validation Report
    report = {
        "experiment": "controlled_known_box_validation",
        "timestamp": time.time(),
        "target_box": {
            "center": target_box.center.tolist(),
            "dimensions": target_box.dimensions.tolist()
        },
        "geometric_validation": geom_metrics,
        "occupancy_grid_3state": occ_stats,
        "obstacles_extracted": [obs.to_dict() for obs in obstacles],
        "safety_assessment": safety_eval,
        "artifacts": {
            "reconstructed_pointcloud": box_pcd_path,
            "reconstructed_mesh": box_mesh_path,
            "occupied_voxels": occ_pcd_path
        }
    }

    report_file = os.path.join(metrics_dir, "box_validation_report.json")
    with open(report_file, "w") as f:
        json.dump(report, f, indent=2)

    print(f"\n================================================================================")
    print(f"  VALIDATION REPORT SAVED TO: {report_file}")
    print(f"================================================================================")

    if args.visualize:
        print("\nLaunching 3D Visualizer...")
        os.system(f"python3 3d_view.py --pcd {box_pcd_path} --mesh {box_mesh_path}")


if __name__ == "__main__":
    main()
