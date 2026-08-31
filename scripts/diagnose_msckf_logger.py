import os
import sys
import yaml
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.dataset_io.synthetic_generator import SyntheticDatasetGenerator
from src.estimator_wrapper.msckf_estimator import MSCKFEstimator
from src.estimator_wrapper.math_utils import rot_to_quat


def log_msckf_corrections(preset: str = "orbit"):
    print("=" * 80)
    print(f"  TASK 4 & 5: DETAILED MSCKF CORRECTION STEP LOGGER ({preset.upper()})")
    print("=" * 80)

    with open("config/synthetic.yaml", "r") as f:
        config = yaml.safe_load(f)

    gen = SyntheticDatasetGenerator(config)
    ds = gen.generate_dataset(preset=preset)
    gt_stream = ds["ground_truth"]
    imu_stream = ds["imu"]
    cam_stream = ds["camera"]

    estimator = MSCKFEstimator(config)
    gt0 = gt_stream[0]
    estimator.p_WB = gt0["position"].copy()
    estimator.v_WB = gt0["velocity"].copy()
    estimator.R_WB = gt0["R_WB"].copy()
    estimator.is_initialized = True

    # Instrument _update_features
    update_log = []

    original_update = estimator._update_features

    def instrumented_update(tracks):
        # Intercept before update
        p_before = estimator.p_WB.copy()
        v_before = estimator.v_WB.copy()
        R_before = estimator.R_WB.copy()
        ba_before = estimator.b_a.copy()
        bg_before = estimator.b_g.copy()

        num_updated = original_update(tracks)

        dp = np.linalg.norm(estimator.p_WB - p_before)
        dv = np.linalg.norm(estimator.v_WB - v_before)
        
        # Geodesic angle change in R_WB
        R_err = R_before.T @ estimator.R_WB
        cos_t = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
        dtheta_deg = np.degrees(np.arccos(cos_t))

        dba = np.linalg.norm(estimator.b_a - ba_before)
        dbg = np.linalg.norm(estimator.b_g - bg_before)

        update_log.append({
            "timestamp": estimator.last_imu_time,
            "tracks_count": len(tracks),
            "features_updated": num_updated,
            "dp_m": dp,
            "dv_mps": dv,
            "dtheta_deg": dtheta_deg,
            "dba": dba,
            "dbg": dbg
        })
        return num_updated

    estimator._update_features = instrumented_update

    imu_idx = 0
    for i, cam in enumerate(cam_stream):
        t_cam = cam["timestamp"]
        while imu_idx < len(imu_stream) and imu_stream[imu_idx]["timestamp"] <= t_cam:
            s = imu_stream[imu_idx]
            estimator.process_imu(s["timestamp"], s["linear_accel"], s["angular_vel"])
            imu_idx += 1
        estimator.process_camera(t_cam, cam["image"])

    print(f"{'Time (s)':<10} | {'Tracks':<8} | {'Upd':<6} | {'||dp|| (m)':<12} | {'||dv|| (m/s)':<14} | {'||dtheta|| (deg)':<16} | {'||dbg||':<10}")
    print("-" * 85)
    for u in update_log[:25]:
        print(f"{u['timestamp']:<10.2f} | {u['tracks_count']:<8} | {u['features_updated']:<6} | {u['dp_m']:<12.4f} | {u['dv_mps']:<14.4f} | {u['dtheta_deg']:<16.4f} | {u['dbg']:<10.6f}")

    dtheta_all = [u["dtheta_deg"] for u in update_log]
    dp_all = [u["dp_m"] for u in update_log]
    print("-" * 85)
    print(f"Max Single-Step Rotation Correction: {np.max(dtheta_all):.3f} deg")
    print(f"Mean Rotation Correction:            {np.mean(dtheta_all):.3f} deg")
    print(f"Max Single-Step Position Correction: {np.max(dp_all):.3f} m")
    print(f"Mean Position Correction:            {np.mean(dp_all):.3f} m")


if __name__ == "__main__":
    log_msckf_corrections()
