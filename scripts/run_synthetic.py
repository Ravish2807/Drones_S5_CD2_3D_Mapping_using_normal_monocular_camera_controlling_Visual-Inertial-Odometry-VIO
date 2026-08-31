import os
import sys
import argparse
import yaml
import json
import numpy as np

# Ensure project root is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from datasets.synthetic.synthetic_generator import SyntheticDatasetGenerator
from interfaces.vio_interface import VIOInterface
from evaluation.metrics import compute_all_metrics
from evaluation.plots import VIOPlotter


def main():
    parser = argparse.ArgumentParser(description="Run Standalone VIO Simulation on Drone Synthetic Trajectories")
    parser.add_argument("--preset", type=str, default="orbit", choices=["hover", "straight", "orbit", "multi_axis", "stress"],
                        help="Synthetic drone flight trajectory preset")
    parser.add_argument("--config", type=str, default="config/synthetic.yaml", help="Path to YAML configuration")
    parser.add_argument("--dropout", action="store_true", help="Inject artificial visual feature dropouts")
    parser.add_argument("--out_dir", type=str, default="results", help="Directory to save outputs")
    args = parser.parse_args()

    print("=" * 70, flush=True)
    print(f"  RUNNING DRONE VIO SIMULATION: PRESET = '{args.preset.upper()}'", flush=True)
    print("=" * 70, flush=True)

    # 1. Load Configuration
    with open(args.config, "r") as f:
        config = yaml.safe_load(f)

    # 2. Generate Synthetic Dataset
    print("[1/5] Generating synthetic drone trajectory and sensor streams...")
    generator = SyntheticDatasetGenerator(config)
    dataset = generator.generate_dataset(preset=args.preset, inject_dropout=args.dropout)

    imu_stream = dataset["imu"]
    cam_stream = dataset["camera"]
    gt_stream = dataset["ground_truth"]

    print(f"      -> IMU samples: {len(imu_stream)} (200 Hz)")
    print(f"      -> Camera frames: {len(cam_stream)} (20 Hz)")
    print(f"      -> 3D Landmarks: {len(dataset['landmarks'])}")

    # 3. Instantiate VIO Interface
    print("[2/5] Initializing Standalone MSCKF VIO Interface...")
    vio = VIOInterface(config)

    # Initialize from t=0 ground truth state for simulation validation
    gt0 = gt_stream[0]
    vio.estimator.p_WB = gt0["position"].copy()
    vio.estimator.v_WB = gt0["velocity"].copy()
    vio.estimator.R_WB = gt0["R_WB"].copy()
    vio.estimator.is_initialized = True

    # Stream sensor events temporally
    print("[3/5] Fusing visual-inertial sensor streams...")
    imu_idx = 0
    cam_idx = 0
    n_imu = len(imu_stream)
    n_cam = len(cam_stream)

    while imu_idx < n_imu or cam_idx < n_cam:
        imu_t = imu_stream[imu_idx]["timestamp"] if imu_idx < n_imu else float("inf")
        cam_t = cam_stream[cam_idx]["timestamp"] if cam_idx < n_cam else float("inf")

        if imu_t <= cam_t and imu_idx < n_imu:
            sample = imu_stream[imu_idx]
            vio.push_imu(sample["timestamp"], *sample["linear_accel"], *sample["angular_vel"])
            imu_idx += 1
        elif cam_idx < n_cam:
            frame = cam_stream[cam_idx]
            vio.push_camera_frame(frame["timestamp"], frame["image"])
            cam_idx += 1

    print(f"      -> Estimation completed! Total snapshots: {len(vio.state_history)}")

    # 4. Evaluate Trajectory Metrics
    print("[4/5] Computing SE(3) trajectory alignment and accuracy metrics...")
    eval_results = compute_all_metrics(vio.state_history, gt_stream)

    ate = eval_results["ate"]
    rpe = eval_results["rpe"]
    vel = eval_results["velocity"]
    perf = eval_results["performance"]

    print("-" * 70)
    print("  EVALUATION SUMMARY RESULTS:")
    print(f"  * ATE RMSE:          {ate['ate_rmse_m']:.4f} m")
    print(f"  * ATE Mean:          {ate['ate_mean_m']:.4f} m")
    print(f"  * ATE Median:        {ate['ate_median_m']:.4f} m")
    print(f"  * ATE Max:           {ate['ate_max_m']:.4f} m")
    print(f"  * RPE Trans RMSE:    {rpe['rpe_trans_rmse_m']:.4f} m")
    print(f"  * Velocity RMSE:     {vel['vel_rmse_mps']:.4f} m/s")
    print(f"  * Mean Latency:      {perf['mean_proc_time_ms']:.2f} ms ({perf['effective_fps']:.1f} FPS)")
    print("-" * 70)

    # 5. Save Results and Figures
    print("[5/5] Generating validation plots and saving metric artifacts...")
    traj_dir = os.path.join(args.out_dir, "trajectories")
    metrics_dir = os.path.join(args.out_dir, "metrics")
    fig_dir = os.path.join(args.out_dir, "figures")
    os.makedirs(traj_dir, exist_ok=True)
    os.makedirs(metrics_dir, exist_ok=True)
    os.makedirs(fig_dir, exist_ok=True)

    seq_name = f"synthetic_{args.preset}"

    # Save trajectories CSV
    traj_csv = os.path.join(traj_dir, f"{seq_name}_estimated_trajectory.csv")
    vio.export_state_csv(traj_csv)

    ori = eval_results["orientation"]

    # Save Metrics JSON
    metrics_json = os.path.join(metrics_dir, f"{seq_name}_metrics.json")
    metrics_export = {
        "sequence": seq_name,
        "preset": args.preset,
        "ate_rmse_m": ate["ate_rmse_m"],
        "ate_mean_m": ate["ate_mean_m"],
        "ate_median_m": ate["ate_median_m"],
        "ate_max_m": ate["ate_max_m"],
        "rpe_trans_rmse_m": rpe["rpe_trans_rmse_m"],
        "ori_rmse_deg": ori["ori_rmse_deg"],
        "ori_final_deg": ori["ori_final_deg"],
        "vel_rmse_mps": vel["vel_rmse_mps"],
        "mean_latency_ms": perf["mean_proc_time_ms"],
        "effective_fps": perf["effective_fps"]
    }
    with open(metrics_json, "w") as f:
        json.dump(metrics_export, f, indent=4)

    # Plot Figures
    plotter = VIOPlotter(output_dir=fig_dir)
    pdash = plotter.plot_summary_dashboard(eval_results, sequence_name=seq_name)

    print(f"      -> Saved trajectory CSV: {traj_csv}")
    print(f"      -> Saved metrics JSON:   {metrics_json}")
    print(f"      -> Saved Dashboard Plot: {pdash}")
    print("=" * 70)
    print("  EXPERIMENT COMPLETE SUCCESSFULLY!")
    print("=" * 70)


if __name__ == "__main__":
    main()
