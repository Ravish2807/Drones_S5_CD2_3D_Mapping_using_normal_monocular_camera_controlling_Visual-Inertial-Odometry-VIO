import os
import sys
import yaml
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.dataset_io.synthetic_generator import SyntheticDatasetGenerator
from src.estimator_wrapper.msckf_estimator import MSCKFEstimator
from src.estimator_wrapper.math_utils import rot_to_quat
from src.evaluation.metrics import compute_all_metrics


def run_isolated_comparison(preset: str = "orbit"):
    print("=" * 80)
    print(f"  ISOLATED 3-WAY COMPARISON: IMU-ONLY vs VISION-ONLY vs FULL-VIO ({preset.upper()})")
    print("=" * 80)

    with open("config/synthetic.yaml", "r") as f:
        config = yaml.safe_load(f)

    # 1. Generate single deterministic dataset (Seed = 42)
    gen = SyntheticDatasetGenerator(config)
    ds = gen.generate_dataset(preset=preset)
    gt_stream = ds["ground_truth"]
    imu_stream = ds["imu"]
    cam_stream = ds["camera"]

    gt0 = gt_stream[0]

    # --- Mode A: IMU-Only (Visual updates disabled) ---
    est_imu_only = MSCKFEstimator(config)
    est_imu_only.p_WB = gt0["position"].copy()
    est_imu_only.v_WB = gt0["velocity"].copy()
    est_imu_only.R_WB = gt0["R_WB"].copy()
    est_imu_only.is_initialized = True

    for s in imu_stream:
        est_imu_only.process_imu(s["timestamp"], s["linear_accel"], s["angular_vel"])
    
    # Sample IMU-only state at camera timestamps
    cam_times = [c["timestamp"] for c in cam_stream]
    imu_only_history = []
    # Re-run synchronized to log at camera timestamps
    est_imu_only = MSCKFEstimator(config)
    est_imu_only.p_WB = gt0["position"].copy()
    est_imu_only.v_WB = gt0["velocity"].copy()
    est_imu_only.R_WB = gt0["R_WB"].copy()
    est_imu_only.is_initialized = True

    imu_idx = 0
    for cam in cam_stream:
        t_cam = cam["timestamp"]
        while imu_idx < len(imu_stream) and imu_stream[imu_idx]["timestamp"] <= t_cam:
            s = imu_stream[imu_idx]
            est_imu_only.process_imu(s["timestamp"], s["linear_accel"], s["angular_vel"])
            imu_idx += 1
        imu_only_history.append({
            "timestamp": t_cam,
            "position": est_imu_only.p_WB.copy(),
            "velocity": est_imu_only.v_WB.copy(),
            "orientation_R": est_imu_only.R_WB.copy(),
            "orientation_q": est_imu_only.propagator.rot_to_quat(est_imu_only.R_WB) if hasattr(est_imu_only.propagator, "rot_to_quat") else np.array([1,0,0,0])
        })

    # --- Mode B: Full VIO (Prediction + MSCKF Correction) ---
    est_full_vio = MSCKFEstimator(config)
    est_full_vio.p_WB = gt0["position"].copy()
    est_full_vio.v_WB = gt0["velocity"].copy()
    est_full_vio.R_WB = gt0["R_WB"].copy()
    est_full_vio.is_initialized = True

    # Instrument MSCKF to log update statistics
    update_logs = []
    original_update = est_full_vio._update_features

    def logged_update(tracks):
        res = original_update(tracks)
        return res

    imu_idx = 0
    for cam in cam_stream:
        t_cam = cam["timestamp"]
        while imu_idx < len(imu_stream) and imu_stream[imu_idx]["timestamp"] <= t_cam:
            s = imu_stream[imu_idx]
            est_full_vio.process_imu(s["timestamp"], s["linear_accel"], s["angular_vel"])
            imu_idx += 1
        est_full_vio.process_camera(t_cam, cam["image"])

    # Compute evaluation metrics
    eval_imu = compute_all_metrics(imu_only_history, gt_stream)
    eval_vio = compute_all_metrics(est_full_vio.trajectory_history, gt_stream)

    print("-" * 80)
    print(f"{'Configuration':<20} | {'ATE RMSE (m)':<15} | {'Ori RMSE (deg)':<15} | {'Vel RMSE (m/s)':<15}")
    print("-" * 80)
    print(f"{'A - IMU Only':<20} | {eval_imu['ate']['ate_rmse_m']:<15.4f} | {eval_imu['orientation']['ori_rmse_deg']:<15.3f} | {eval_imu['velocity']['vel_rmse_mps']:<15.4f}")
    print(f"{'C - Full VIO':<20} | {eval_vio['ate']['ate_rmse_m']:<15.4f} | {eval_vio['orientation']['ori_rmse_deg']:<15.3f} | {eval_vio['velocity']['vel_rmse_mps']:<15.4f}")
    print("-" * 80)


if __name__ == "__main__":
    run_isolated_comparison()
