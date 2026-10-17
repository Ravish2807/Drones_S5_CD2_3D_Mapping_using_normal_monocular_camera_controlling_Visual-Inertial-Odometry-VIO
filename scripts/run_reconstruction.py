#!/usr/bin/env python3
"""
Phase 3 VIO-Guided 3D Reconstruction Pipeline Runner.
Demonstrates the full end-to-end pipeline:
  Camera + IMU -> MSCKF VIO -> T_WC(t) -> Keyframe Selection ->
  Multi-View Stereo (MVS) -> Depth Map Fusion -> Dense Point Cloud -> Surface Mesh
"""

import os
import sys
import yaml
import json
import time
import argparse
import cv2
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

# Ensure workspace root is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from datasets.synthetic.synthetic_generator import SyntheticDatasetGenerator
from interfaces.vio_interface import VIOInterface
from trajectory.camera_pose import CameraPose, CameraExtrinsics
from reconstruction.keyframe_bundle import Keyframe, CameraIntrinsics, KeyframeDatabase
from reconstruction.keyframe_selector import KeyframeSelector
from reconstruction.mvs_depth_estimator import MVSDepthEstimator
from reconstruction.depth_fusion import DepthFusion
from reconstruction.surface_mesher import SurfaceMesher


def create_synthetic_target_object(center: np.ndarray = np.array([0.0, 0.0, 2.0]), radius: float = 0.8, num_points: int = 1200):
    """
    Creates a textured 3D target object (sphere + cylindrical pedestal) at the scene center.
    This serves as the target object being scanned and reconstructed by the orbiting drone.
    """
    np.random.seed(42)
    # Sphere surface points
    phi = np.random.uniform(0, 2 * np.pi, num_points // 2)
    costheta = np.random.uniform(-1, 1, num_points // 2)
    u = np.random.uniform(0, 1, num_points // 2)

    theta = np.arccos(costheta)
    r = radius * np.cbrt(u)
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    sphere_pts = np.column_stack([x, y, z]) + center

    # Cylinder base points
    n_cyl = num_points // 2
    theta_cyl = np.random.uniform(0, 2 * np.pi, n_cyl)
    r_cyl = np.random.uniform(0.1, radius * 0.9, n_cyl)
    z_cyl = np.random.uniform(-radius * 1.2, 0.0, n_cyl)
    x_cyl = r_cyl * np.cos(theta_cyl)
    y_cyl = r_cyl * np.sin(theta_cyl)
    cyl_pts = np.column_stack([x_cyl, y_cyl, z_cyl]) + center

    return np.vstack([sphere_pts, cyl_pts])


def main():
    parser = argparse.ArgumentParser(description="Phase 3 VIO-Guided 3D Reconstruction Pipeline")
    parser.add_argument("--config", type=str, default="config/synthetic.yaml", help="Path to YAML configuration")
    parser.add_argument("--preset", type=str, default="orbit", choices=["orbit", "multi_axis"], help="Drone flight preset")
    parser.add_argument("--out_dir", type=str, default="results", help="Directory to save outputs")
    parser.add_argument("--visualize", action="store_true", help="Launch interactive 3D Open3D viewer upon completion")
    args = parser.parse_args()

    print("=" * 80)
    print("  PHASE 3 — VIO-GUIDED 3D RECONSTRUCTION PIPELINE")
    print("  Camera + IMU -> MSCKF VIO -> T_WC -> Keyframes -> MVS -> Dense 3D Mesh")
    print("=" * 80)

    # 1. Load Configuration
    with open(args.config, "r") as f:
        config = yaml.safe_load(f)

    out_dir = args.out_dir
    kf_dir = os.path.join(out_dir, "keyframes")
    pcd_dir = os.path.join(out_dir, "pointclouds")
    depth_dir = os.path.join(out_dir, "depth_maps")
    plots_dir = os.path.join(out_dir, "plots")
    os.makedirs(kf_dir, exist_ok=True)
    os.makedirs(pcd_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(plots_dir, exist_ok=True)

    # 2. Setup Camera Intrinsics & Extrinsics
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

    # 3. Generate Flight Dataset with Central 3D Target Object
    print("\n[Step 3.1] Generating 6-DoF flight stream & central 3D target object...")
    generator = SyntheticDatasetGenerator(config)

    # Inject structured 3D target object at scene center [0, 0, 2.0]
    target_obj_pts = create_synthetic_target_object(center=np.array([0.0, 0.0, 2.0]), radius=0.75, num_points=800)
    generator.landmarks_W = np.vstack([generator.landmarks_W, target_obj_pts])

    dataset = generator.generate_dataset(preset=args.preset)
    imu_stream = dataset["imu"]
    cam_stream = dataset["camera"]
    gt_stream = dataset["ground_truth"]

    print(f"           -> Total IMU packets:    {len(imu_stream)} (200 Hz)")
    print(f"           -> Total Camera frames:  {len(cam_stream)} (20 Hz)")
    print(f"           -> Scene Landmarks:      {len(generator.landmarks_W)} (including central 3D object)")

    # 4. Initialize MSCKF VIO Engine
    print("\n[Step 3.2] Executing MSCKF VIO fast loop & tracking camera pose T_WC(t)...")
    vio = VIOInterface(config)

    gt0 = gt_stream[0]
    vio.estimator.p_WB = gt0["position"].copy()
    vio.estimator.v_WB = gt0["velocity"].copy()
    vio.estimator.R_WB = gt0["R_WB"].copy()
    vio.estimator.is_initialized = True

    # 5. Initialize Keyframe Selector & Database
    selector = KeyframeSelector(
        min_translation_m=0.50,
        min_rotation_deg=15.0,
        min_feature_overlap=0.55,
        max_time_interval_sec=3.0
    )
    db = KeyframeDatabase()

    # Stream Sensor Events
    imu_idx = 0
    cam_idx = 0
    n_imu = len(imu_stream)
    n_cam = len(cam_stream)

    t0_start = time.time()
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

                # Rigorous SE(3) Extrinsic Transformation: T_WC = T_WB * T_BC
                p_WC = p_WB + R_WB @ extrinsics.p_BC
                R_WC = R_WB @ extrinsics.R_BC
                T_WC = np.eye(4)
                T_WC[0:3, 0:3] = R_WC
                T_WC[0:3, 3] = p_WC

                # Keyframe Selection Check
                feature_ids = None
                feature_pts = None
                if hasattr(vio.estimator, "tracker") and vio.estimator.tracker.prev_ids is not None:
                    feature_ids = list(vio.estimator.tracker.prev_ids)
                    feature_pts = vio.estimator.tracker.prev_pts

                is_kf, reason, kf = selector.evaluate(
                    timestamp=timestamp,
                    image=img,
                    T_WC=T_WC,
                    intrinsics=intrinsics,
                    feature_ids=feature_ids,
                    feature_points=feature_pts,
                    tracking_quality={"num_features": len(feature_ids) if feature_ids else 0}
                )

                if is_kf and kf is not None:
                    db.add(kf)

            cam_idx += 1

    elapsed_vio = time.time() - t0_start
    print(f"           -> VIO completed in {elapsed_vio:.2f}s (Average rate: {n_cam/elapsed_vio:.1f} FPS)")
    print(f"           -> Total frames evaluated: {selector.total_frames_evaluated}")
    print(f"           -> Keyframes selected:     {len(db)} keyframes ({len(db)/selector.total_frames_evaluated*100:.1f}% reduction)")

    # 6. Save Keyframe Bundle to Disk
    print("\n[Step 3.3] Saving Keyframe Bundle (Images, Intrinsics, Poses T_WC)...")
    db.save_to_disk(kf_dir)

    # 7. Multi-View Stereo Depth Estimation
    print("\n[Step 3.4 & 3.5] Multi-View Stereo (MVS) Depth Estimation on Keyframes...")
    depth_estimator = MVSDepthEstimator(
        min_depth=1.0,
        max_depth=7.0,
        num_depth_planes=48,
        patch_size=5,
        method="plane_sweep"
    )

    for i in range(len(db)):
        depth_map, conf_map = depth_estimator.process_keyframe(i, db.keyframes)
        # Save sample depth map visualization
        if i % max(1, len(db) // 4) == 0:
            depth_vis = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            depth_vis_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)
            cv2.imwrite(os.path.join(depth_dir, f"depth_vis_kf_{i:04d}.png"), depth_vis_color)

    print(f"           -> Computed dense depth maps for all {len(db)} keyframes.")

    # 8. Depth Map Fusion into Dense 3D Point Cloud
    print("\n[Step 3.6 & 3.7] Multi-View Depth Map Fusion & Outlier Rejection...")
    fusion = DepthFusion(voxel_size_m=0.06, min_depth=1.0, max_depth=7.0, confidence_thresh=0.35)

    # Backproject and fuse
    dense_pcd = fusion.fuse_point_clouds(db.keyframes)

    dense_pcd_path = os.path.join(pcd_dir, "dense_reconstruction.ply")
    fusion.export_point_cloud(dense_pcd, dense_pcd_path)
    print(f"           -> Dense point cloud saved: {dense_pcd_path} ({len(dense_pcd.points)} points)")

    # 9. Surface Reconstruction (Poisson 3D Mesh)
    print("\n[Step 3.8] 3D Surface Reconstruction (Poisson Meshing)...")
    mesher = SurfaceMesher(poisson_depth=7, density_quantile_trim=0.05)
    dense_mesh = mesher.reconstruct_poisson_mesh(dense_pcd)

    dense_mesh_path = os.path.join(pcd_dir, "dense_reconstruction_mesh.ply")
    mesher.export_mesh(dense_mesh, dense_mesh_path)

    # Obstacle Voxel Grid
    voxel_path = os.path.join(pcd_dir, "dense_reconstruction_voxels.ply")
    mesher.export_voxel_grid(dense_pcd, voxel_path, voxel_size=0.10)

    # 10. Generate Interactive 3D Moveable HTML Graph (Plotly)
    print("\n[Step 3.9] Generating Standalone 3D Moveable HTML Graph...")
    from scripts.generate_interactive_3d import generate_interactive_3d_plot
    interactive_html = os.path.join(plots_dir, "interactive_3d_reconstruction.html")
    generate_interactive_3d_plot(
        csv_path=os.path.join(out_dir, "trajectories", f"synthetic_{args.preset}_camera_trajectory.csv"),
        ply_path=dense_pcd_path,
        output_html=interactive_html
    )

    # 11. Compile Summary JSON Report
    report = {
        "preset": args.preset,
        "total_frames": selector.total_frames_evaluated,
        "keyframes_selected": len(db),
        "keyframe_compression_ratio": float(len(db) / max(1, selector.total_frames_evaluated)),
        "dense_points_reconstructed": len(dense_pcd.points),
        "mesh_triangles": len(dense_mesh.triangles),
        "mesh_vertices": len(dense_mesh.vertices),
        "dense_pointcloud_path": dense_pcd_path,
        "mesh_path": dense_mesh_path,
        "interactive_html_path": interactive_html
    }
    report_path = os.path.join(out_dir, "metrics", "phase3_reconstruction_summary.json")
    os.makedirs(os.path.dirname(report_path), exist_ok=True)
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)

    print("\n" + "=" * 80)
    print("  PHASE 3 RECONSTRUCTION SUMMARY:")
    print(f"  * Total Processed Camera Frames:   {report['total_frames']}")
    print(f"  * Keyframes Selected:             {report['keyframes_selected']}")
    print(f"  * Dense Point Cloud:               {report['dense_points_reconstructed']} points")
    print(f"  * 3D Surface Mesh:                 {report['mesh_triangles']} triangles, {report['mesh_vertices']} vertices")
    print(f"  * Output Mesh File:                {dense_mesh_path}")
    print(f"  * Interactive Moveable 3D Graph:   {interactive_html}")
    print("=" * 80)

    if args.visualize:
        print("\nLaunching 3D Open3D Viewer...")
        os.system(f"python3 3d_view.py --pcd {dense_pcd_path} --mesh {dense_mesh_path}")


if __name__ == "__main__":
    main()
