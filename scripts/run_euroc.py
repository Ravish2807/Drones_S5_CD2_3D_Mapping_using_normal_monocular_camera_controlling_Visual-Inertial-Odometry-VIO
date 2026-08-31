import os
import sys
import argparse
import yaml
import json
import numpy as np

# Ensure project root is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.dataset_io.euroc_loader import EuRoCDatasetLoader
from src.estimator_wrapper.msckf_estimator import MSCKFEstimator
from src.evaluation.metrics import compute_all_metrics
from src.visualization.plotter import VIOPlotter


def main():
    parser = argparse.ArgumentParser(description="Run Standalone VIO Estimator on EuRoC MAV Dataset")
    parser.add_argument("--sequence_path", type=str, required=True, help="Path to EuRoC sequence folder (e.g. data/MH_01_easy)")
    parser.add_argument("--config", type=str, default="config/euroc.yaml", help="Path to YAML configuration")
    parser.add_argument("--out_dir", type=str, default="results", help="Directory to save outputs")
    parser.add_argument("--max_frames", type=int, default=None, help="Optional frame limit for quick testing")
    args = parser.parse_args()

    seq_name = os.path.basename(os.path.normpath(args.sequence_path))
    print("=" * 70)
    print(f"  RUNNING EUROC MAV VIO BENCHMARK: SEQUENCE = '{seq_name.upper()}'")
    print("=" * 70)

    if not os.path.exists(args.sequence_path):
        print(f"[ERROR] Sequence directory not found: {args.sequence_path}")
        print("Please download and extract EuRoC MAV sequences to the data/ directory.")
        print("Refer to docs/dataset_notes.md for instructions.")
        sys.exit(1)

    # 1. Load Configuration
    with open(args.config, "r") as f:
        config = yaml.safe_load(f)

    # 2. Ingest Dataset
    print("[1/5] Loading EuRoC MAV sequence...")
    loader = EuRoCDatasetLoader(args.sequence_path)
    gt_df = loader.get_ground_truth()

    # 3. Instantiate Estimator
    print("[2/5] Initializing MSCKF visual-inertial estimator...")
    estimator = MSCKFEstimator(config)

    # 4. Stream events
    print("[3/5] Fusing visual-inertial sensor stream...")
    frame_count = 0
    for event in loader.stream_sensor_events():
        if event["type"] == "imu":
            estimator.process_imu(event["timestamp"], event["linear_accel"], event["angular_vel"])
        elif event["type"] == "camera":
            if event["image"] is not None:
                estimator.process_camera(event["timestamp"], event["image"])
                frame_count += 1
                if frame_count % 50 == 0:
                    print(f"      -> Processed {frame_count} camera frames...")
                if args.max_frames and frame_count >= args.max_frames:
                    break

    print(f"      -> Estimation completed! Total frames processed: {frame_count}")

    # 5. Evaluate against Ground Truth
    print("[4/5] Computing SE(3) trajectory alignment and metrics...")
    if gt_df is not None:
        gt_records = []
        for _, row in gt_df.iterrows():
            gt_records.append({
                "timestamp": row["timestamp_s"],
                "position": np.array([row["p_x"], row["p_y"], row["p_z"]]),
                "velocity": np.array([row["v_x"], row["v_y"], row["v_z"]]),
                "orientation_q": np.array([row["q_w"], row["q_x"], row["q_y"], row["q_z"]])
            })

        eval_results = compute_all_metrics(estimator.trajectory_history, gt_records)

        ate = eval_results["ate"]
        rpe = eval_results["rpe"]
        vel = eval_results["velocity"]
        perf = eval_results["performance"]

        print("-" * 70)
        print(f"  EUROC [{seq_name.upper()}] EVALUATION SUMMARY:")
        print(f"  * ATE RMSE:          {ate['ate_rmse_m']:.4f} m")
        print(f"  * ATE Mean:          {ate['ate_mean_m']:.4f} m")
        print(f"  * RPE Trans RMSE:    {rpe['rpe_trans_rmse_m']:.4f} m")
        print(f"  * Velocity RMSE:     {vel['vel_rmse_mps']:.4f} m/s")
        print(f"  * Mean Latency:      {perf['mean_proc_time_ms']:.2f} ms ({perf['effective_fps']:.1f} FPS)")
        print("-" * 70)

        # 6. Save Results & Figures
        print("[5/5] Generating validation plots and saving artifacts...")
        traj_dir = os.path.join(args.out_dir, "trajectories")
        metrics_dir = os.path.join(args.out_dir, "metrics")
        fig_dir = os.path.join(args.out_dir, "figures")
        os.makedirs(traj_dir, exist_ok=True)
        os.makedirs(metrics_dir, exist_ok=True)
        os.makedirs(fig_dir, exist_ok=True)

        plotter = VIOPlotter(output_dir=fig_dir)
        p3d = plotter.plot_3d_trajectory(eval_results, sequence_name=f"euroc_{seq_name}")
        pxyz = plotter.plot_position_components(eval_results, sequence_name=f"euroc_{seq_name}")
        perr = plotter.plot_errors_and_residuals(eval_results, sequence_name=f"euroc_{seq_name}")
        pvel = plotter.plot_velocities(eval_results, sequence_name=f"euroc_{seq_name}")
        pdash = plotter.plot_comprehensive_dashboard(eval_results, estimator.trajectory_history, sequence_name=f"euroc_{seq_name}")

        metrics_json = os.path.join(metrics_dir, f"euroc_{seq_name}_metrics.json")
        with open(metrics_json, "w") as f:
            json.dump({
                "sequence": f"euroc_{seq_name}",
                "ate_rmse_m": ate["ate_rmse_m"],
                "ate_mean_m": ate["ate_mean_m"],
                "rpe_trans_rmse_m": rpe["rpe_trans_rmse_m"],
                "vel_rmse_mps": vel["vel_rmse_mps"],
                "mean_latency_ms": perf["mean_proc_time_ms"],
                "effective_fps": perf["effective_fps"]
            }, f, indent=4)

        print(f"      -> Metrics written to: {metrics_json}")
        print(f"      -> Dashboard plot:     {pdash}")
    else:
        print("[WARNING] No ground truth file found in sequence directory. Trajectory saved without metrics.")

    print("=" * 70)
    print("  EUROC BENCHMARK RUN COMPLETE!")
    print("=" * 70)


if __name__ == "__main__":
    main()
