import os
import sys
import yaml
import json
import argparse
import pandas as pd
import numpy as np

# Ensure project root is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from datasets.synthetic.synthetic_generator import SyntheticDatasetGenerator
from interfaces.vio_interface import VIOInterface
from trajectory.camera_pose import (
    BodyPose,
    CameraExtrinsics,
    CameraPose,
    CameraTrajectory,
    transform_body_to_camera_pose,
)
from trajectory.trajectory_exporter import TrajectoryExporter
from trajectory.trajectory_metrics import (
    compute_camera_trajectory_metrics,
    plot_camera_trajectory_comparison,
)


def run_synthetic_benchmark(
    preset: str = "orbit",
    config_path: str = "config/synthetic.yaml",
    out_dir: str = "results",
    dropout: bool = False
) -> dict:
    """
    Executes Phase 2 Camera Trajectory benchmark for a specific preset:
    1. Generates synthetic IMU and Camera streams with ground truth T_WC.
    2. Runs MSCKF-VIO to estimate T_WB.
    3. Transforms T_WB -> T_WC using camera extrinsics T_BC.
    4. Validates 1-to-1 timestamp alignment.
    5. Computes Camera ATE, RPE, Orientation, and Velocity metrics.
    6. Generates 3D plots, error dashboard, and exports TUM/CSV trajectories.
    """
    print("=" * 75, flush=True)
    print(f"  PHASE 2: CAMERA TRAJECTORY VALIDATION (PRESET = {preset.upper()})", flush=True)
    print("=" * 75, flush=True)

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # 1. Dataset Generation
    generator = SyntheticDatasetGenerator(config)
    dataset = generator.generate_dataset(preset=preset, inject_dropout=dropout)

    imu_stream = dataset["imu"]
    cam_stream = dataset["camera"]
    gt_stream = dataset["ground_truth"]

    print(f"[1/5] Loaded Synthetic Data: {len(imu_stream)} IMU samples, {len(cam_stream)} Camera frames.")

    # 2. Extract Ground Truth Camera Trajectory T_WC^{GT}
    gt_cam_poses = []
    cam_timestamps = []
    for frame in cam_stream:
        t = frame["timestamp"]
        cam_timestamps.append(t)
        gt_cam_poses.append(
            CameraPose(timestamp=t, p_WC=frame["p_WC"].copy(), R_WC=frame["R_WC"].copy(),
                       frame_idx=frame["frame_idx"], source="ground_truth")
        )
    gt_cam_traj = CameraTrajectory(gt_cam_poses, name=f"{preset}_camera_gt")

    # 3. Initialize VIO Estimator
    extrinsics = CameraExtrinsics.from_config(config)
    vio = VIOInterface(config)

    # Initialize from t=0 ground truth state
    gt0 = gt_stream[0]
    vio.estimator.p_WB = gt0["position"].copy()
    vio.estimator.v_WB = gt0["velocity"].copy()
    vio.estimator.R_WB = gt0["R_WB"].copy()
    vio.estimator.is_initialized = True

    print("[2/5] Fusing sensor streams through MSCKF-VIO Core...")
    imu_idx = 0
    cam_idx = 0
    n_imu = len(imu_stream)
    n_cam = len(cam_stream)

    est_cam_poses = []

    while imu_idx < n_imu or cam_idx < n_cam:
        imu_t = imu_stream[imu_idx]["timestamp"] if imu_idx < n_imu else float("inf")
        cam_t = cam_stream[cam_idx]["timestamp"] if cam_idx < n_cam else float("inf")

        if imu_t <= cam_t and imu_idx < n_imu:
            sample = imu_stream[imu_idx]
            vio.push_imu(sample["timestamp"], *sample["linear_accel"], *sample["angular_vel"])
            imu_idx += 1
        elif cam_idx < n_cam:
            frame = cam_stream[cam_idx]
            state = vio.push_camera_frame(frame["timestamp"], frame["image"])

            if state is not None:
                # Step 2.3 & 2.4: Convert Body Pose T_WB -> Camera Pose T_WC
                body_pose = BodyPose.from_vio_state(state)
                cam_pose = transform_body_to_camera_pose(body_pose, extrinsics, frame_idx=frame["frame_idx"])
                est_cam_poses.append(cam_pose)

            cam_idx += 1

    est_cam_traj = CameraTrajectory(est_cam_poses, name=f"{preset}_camera_est")
    print(f"      -> Estimation complete. Generated {len(est_cam_traj)} Camera Poses (T_WC).")

    # 4. Step 2.9: Validate Timestamp Alignment
    print("[3/5] Validating 1-to-1 timestamp alignment...")
    alignment_report = est_cam_traj.validate_timestamp_alignment(cam_timestamps, max_allowed_dt=1e-4)
    print(f"      -> Matched: {alignment_report['matched_frames']}/{alignment_report['total_frames']} frames")
    print(f"      -> Max Delta t: {alignment_report['max_delta_t_sec']*1e3:.4f} ms, Mean Delta t: {alignment_report['mean_delta_t_sec']*1e3:.4f} ms")

    # 5. Step 2.8: Compute Camera Trajectory Metrics
    print("[4/5] Computing camera trajectory metrics (ATE, RPE, Ori, Vel)...")
    cam_metrics = compute_camera_trajectory_metrics(est_cam_traj, gt_cam_traj, alignment_mode="se3", delta_sec=1.0)

    ate = cam_metrics["ate"]
    rpe = cam_metrics["rpe"]
    ori = cam_metrics["orientation"]
    vel = cam_metrics["velocity"]

    print("-" * 75)
    print(f"  CAMERA TRAJECTORY EVALUATION: [{preset.upper()}]")
    print(f"  * Camera ATE RMSE:       {ate['rmse_m']:.4f} m (Mean: {ate['mean_m']:.4f} m, Max: {ate['max_m']:.4f} m)")
    print(f"  * Camera RPE Trans RMSE: {rpe['trans_rmse_m']:.4f} m/s")
    print(f"  * Camera RPE Rot RMSE:   {rpe['rot_rmse_deg']:.2f} deg/s")
    print(f"  * Orientation RMSE:      {ori['rmse_deg']:.2f} deg (Final: {ori['final_deg']:.2f} deg)")
    print(f"  * Velocity RMSE:         {vel['rmse_mps']:.4f} m/s")
    print("-" * 75)

    # 6. Save Artifacts (Trajectories, Plots, Metrics)
    print("[5/5] Exporting trajectory artifacts, validation plots, and summary logs...")
    traj_dir = os.path.join(out_dir, "trajectories")
    plot_dir = os.path.join(out_dir, "plots")
    metrics_dir = os.path.join(out_dir, "metrics")

    os.makedirs(traj_dir, exist_ok=True)
    os.makedirs(plot_dir, exist_ok=True)
    os.makedirs(metrics_dir, exist_ok=True)

    seq_name = f"synthetic_{preset}"

    # Export standardized trajectories (TUM and CSV)
    csv_path = os.path.join(traj_dir, f"{seq_name}_camera_trajectory.csv")
    tum_path = os.path.join(traj_dir, f"{seq_name}_camera_trajectory.tum")
    align_csv = os.path.join(traj_dir, f"{seq_name}_timestamp_alignment.csv")

    TrajectoryExporter.export_csv(est_cam_traj, csv_path)
    TrajectoryExporter.export_tum(est_cam_traj, tum_path)
    TrajectoryExporter.export_alignment_log(alignment_report, align_csv)

    # Export Plot Figures
    fig3d, figdash = plot_camera_trajectory_comparison(cam_metrics, plot_dir, seq_name)

    # Export Metrics JSON
    json_path = os.path.join(metrics_dir, f"{seq_name}_camera_metrics.json")
    metrics_record = {
        "sequence": seq_name,
        "preset": preset,
        "camera_ate_rmse_m": ate["rmse_m"],
        "camera_ate_mean_m": ate["mean_m"],
        "camera_ate_median_m": ate["median_m"],
        "camera_ate_max_m": ate["max_m"],
        "camera_rpe_trans_rmse_m": rpe["trans_rmse_m"],
        "camera_rpe_rot_rmse_deg": rpe["rot_rmse_deg"],
        "camera_ori_rmse_deg": ori["rmse_deg"],
        "camera_ori_final_deg": ori["final_deg"],
        "camera_vel_rmse_mps": vel["rmse_mps"],
        "max_delta_t_ms": alignment_report["max_delta_t_sec"] * 1000.0,
        "mean_delta_t_ms": alignment_report["mean_delta_t_sec"] * 1000.0
    }
    with open(json_path, "w") as f:
        json.dump(metrics_record, f, indent=4)

    return metrics_record


def run_all_synthetic_benchmarks(
    config_path: str = "config/synthetic.yaml",
    out_dir: str = "results"
) -> pd.DataFrame:
    """
    Step 2.10: Runs all 5 synthetic presets and compiles the master summary table.
    Presets: hover, straight, orbit, multi_axis, stress
    """
    presets = ["hover", "straight", "orbit", "multi_axis", "stress"]
    records = []

    for p in presets:
        dropout = (p == "stress")
        rec = run_synthetic_benchmark(preset=p, config_path=config_path, out_dir=out_dir, dropout=dropout)
        records.append(rec)

    # Compile Summary DataFrame as requested in Step 2.10:
    # Columns: Sequence, Camera ATE, RPE, Orientation, Velocity
    summary_df = pd.DataFrame([{
        "Sequence": r["preset"].capitalize(),
        "Camera ATE (m)": f"{r['camera_ate_rmse_m']:.4f}",
        "RPE (m)": f"{r['camera_rpe_trans_rmse_m']:.4f}",
        "Orientation (deg)": f"{r['camera_ori_rmse_deg']:.2f}",
        "Velocity (m/s)": f"{r['camera_vel_rmse_mps']:.4f}"
    } for r in records])

    summary_csv = os.path.join(out_dir, "summary.csv")
    summary_df.to_csv(summary_csv, index=False)

    print("\n" + "=" * 75)
    print("  PHASE 2 MASTER SYNTHETIC BENCHMARK SUMMARY TABLE:")
    print("=" * 75)
    print(summary_df.to_string(index=False))
    print(f"\nSaved master summary CSV to: {summary_csv}")
    print("=" * 75 + "\n")

    return summary_df


def main():
    parser = argparse.ArgumentParser(description="Phase 2: Camera Trajectory Validation")
    parser.add_argument("--preset", type=str, default="all", choices=["all", "hover", "straight", "orbit", "multi_axis", "stress"],
                        help="Synthetic trajectory preset (or 'all' for complete benchmark suite)")
    parser.add_argument("--config", type=str, default="config/synthetic.yaml", help="Path to config YAML")
    parser.add_argument("--out_dir", type=str, default="results", help="Directory to save outputs")
    args = parser.parse_args()

    if args.preset == "all":
        run_all_synthetic_benchmarks(config_path=args.config, out_dir=args.out_dir)
    else:
        run_synthetic_benchmark(preset=args.preset, config_path=args.config, out_dir=args.out_dir, dropout=(args.preset == "stress"))


if __name__ == "__main__":
    main()
