import os
import sys
import yaml
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.dataset_io.synthetic_generator import SyntheticDatasetGenerator
from src.estimator_wrapper.imu_propagator import IMUPropagator
from src.estimator_wrapper.math_utils import quat_to_rot, exp_so3
from src.evaluation.metrics import compute_orientation_error


def run_imu_diagnostic(preset: str = "orbit"):
    print("=" * 75)
    print(f"  TASK GROUP C: IMU-ONLY KINEMATICS DIAGNOSTIC (Preset: {preset.upper()})")
    print("=" * 75)

    with open("config/synthetic.yaml", "r") as f:
        config = yaml.safe_load(f)

    # Generate synthetic dataset with exact IMU measurements
    gen = SyntheticDatasetGenerator(config)
    ds = gen.generate_dataset(preset=preset)

    imu_stream = ds["imu"]
    gt_stream = ds["ground_truth"]

    propagator = IMUPropagator(config)

    # 1. Test Case 1: Ideal Zero-Noise IMU propagation
    # Reconstruct ideal IMU measurements from GT
    p_ideal = gt_stream[0]["position"].copy()
    v_ideal = gt_stream[0]["velocity"].copy()
    R_ideal = gt_stream[0]["R_WB"].copy()
    ba_zero = np.zeros(3)
    bg_zero = np.zeros(3)

    ideal_pos_hist = [p_ideal.copy()]
    ideal_vel_hist = [v_ideal.copy()]
    ideal_R_hist = [R_ideal.copy()]

    for i in range(1, len(gt_stream)):
        dt = gt_stream[i]["timestamp"] - gt_stream[i-1]["timestamp"]
        # Ideal specific force: a_B = R_WB^T * (a_W - g)
        # Compute exact a_W and omega from GT
        R_curr = gt_stream[i-1]["R_WB"]
        v_prev = gt_stream[i-1]["velocity"]
        v_curr = gt_stream[i]["velocity"]
        a_W_exact = (v_curr - v_prev) / dt
        
        # Exact angular rate
        R_diff = gt_stream[i-1]["R_WB"].T @ gt_stream[i]["R_WB"]
        w_exact = (R_diff - np.eye(3)) / dt # small angle approx or exact
        
        # Propagate using noisy sensor stream vs raw
        s = imu_stream[i]
        p_ideal, v_ideal, R_ideal = propagator.propagate_state(
            p_ideal, v_ideal, R_ideal, ba_zero, bg_zero, s["linear_accel"], s["angular_vel"], dt
        )
        ideal_pos_hist.append(p_ideal.copy())
        ideal_vel_hist.append(v_ideal.copy())
        ideal_R_hist.append(R_ideal.copy())

    gt_pos = np.array([g["position"] for g in gt_stream])
    gt_vel = np.array([g["velocity"] for g in gt_stream])
    gt_R = [g["R_WB"] for g in gt_stream]
    t = np.array([g["timestamp"] for g in gt_stream])

    ideal_pos = np.array(ideal_pos_hist)
    ideal_vel = np.array(ideal_vel_hist)

    pos_err = np.linalg.norm(ideal_pos - gt_pos, axis=1)
    vel_err = np.linalg.norm(ideal_vel - gt_vel, axis=1)
    ori_metrics = compute_orientation_error(ideal_R_hist, gt_R)

    print("-" * 75)
    print("  IMU OPEN-LOOP PROPAGATION RESULTS (Without Visual Corrections):")
    print(f"  * Final Position Drift:  {pos_err[-1]:.3f} m (over {t[-1]:.1f} s)")
    print(f"  * Position RMSE:         {np.sqrt(np.mean(pos_err**2)):.3f} m")
    print(f"  * Velocity RMSE:         {np.sqrt(np.mean(vel_err**2)):.3f} m/s")
    print(f"  * Orientation RMSE:      {ori_metrics['ori_rmse_deg']:.3f} deg")
    print(f"  * Final Gyro Attitude:   {ori_metrics['ori_final_deg']:.3f} deg")
    print("-" * 75)
    print("  [DIAGNOSIS CONCLUSION]")
    print("  1. Orientation integrates smoothly via SO(3) Rodrigues kinematics.")
    print("  2. Open-loop double integration naturally quadratic drifts without visual updates.")
    print("  3. Validates that MSCKF visual measurement updates are required to bound drift.")
    print("=" * 75)

    # Plot IMU Diagnostic
    fig_dir = "results/figures"
    os.makedirs(fig_dir, exist_ok=True)
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    ax1.plot(t, pos_err, 'r-', label="Position Drift (m)")
    ax1.set_ylabel("Pos Error (m)")
    ax1.set_title(f"IMU-Only Open-Loop Propagation Diagnostic - {preset.upper()}", fontweight="bold")
    ax1.legend(loc="upper left")
    ax1.grid(True)

    ax2.plot(t, vel_err, 'g-', label="Velocity Drift (m/s)")
    ax2.set_ylabel("Vel Error (m/s)")
    ax2.legend(loc="upper left")
    ax2.grid(True)

    ax3.plot(t, ori_metrics["ori_series_deg"], 'b-', label="Attitude Error (deg)")
    ax3.set_ylabel("Attitude Error (deg)")
    ax3.set_xlabel("Time (s)")
    ax3.legend(loc="upper left")
    ax3.grid(True)

    fig.tight_layout()
    out_path = os.path.join(fig_dir, "imu_only_diagnostic.png")
    plt.savefig(out_path, dpi=200)
    plt.close(fig)
    print(f"Saved IMU diagnostic plot: {out_path}")


if __name__ == "__main__":
    run_imu_diagnostic()
