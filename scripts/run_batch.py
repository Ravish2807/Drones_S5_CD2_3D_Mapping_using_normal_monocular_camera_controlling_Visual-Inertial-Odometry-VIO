import sys
import os
import yaml
import json
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.dataset_io.synthetic_generator import SyntheticDatasetGenerator
from src.estimator_wrapper.msckf_estimator import MSCKFEstimator
from src.evaluation.metrics import compute_all_metrics
from src.visualization.plotter import VIOPlotter


def run_benchmark(preset: str, dropout: bool = False):
    print(f"\n======================================================================")
    print(f"  RUNNING BENCHMARK PRESET: {preset.upper()} (dropout={dropout})")
    print(f"======================================================================")

    with open("config/synthetic.yaml", "r") as f:
        config = yaml.safe_load(f)

    # 1. Dataset
    generator = SyntheticDatasetGenerator(config)
    dataset = generator.generate_dataset(preset=preset, inject_dropout=dropout)

    imu_stream = dataset["imu"]
    cam_stream = dataset["camera"]
    gt_stream = dataset["ground_truth"]

    print(f"Generated: IMU={len(imu_stream)}, Camera={len(cam_stream)}, Landmarks={len(dataset['landmarks'])}")

    # 2. Estimator
    estimator = MSCKFEstimator(config)
    gt0 = gt_stream[0]
    estimator.p_WB = gt0["position"].copy()
    estimator.v_WB = gt0["velocity"].copy()
    estimator.R_WB = gt0["R_WB"].copy()
    estimator.is_initialized = True

    # 3. Process
    imu_idx = 0
    t0 = time.time()
    for i, cam in enumerate(cam_stream):
        t_cam = cam["timestamp"]
        while imu_idx < len(imu_stream) and imu_stream[imu_idx]["timestamp"] <= t_cam:
            s = imu_stream[imu_idx]
            estimator.process_imu(s["timestamp"], s["linear_accel"], s["angular_vel"])
            imu_idx += 1
        estimator.process_camera(t_cam, cam["image"])
        if (i + 1) % 100 == 0:
            print(f"  -> Processed {i + 1}/{len(cam_stream)} frames...")

    total_time = time.time() - t0
    print(f"Estimation finished in {total_time:.2f}s ({len(cam_stream)/total_time:.1f} FPS)")

    # 4. Metrics
    eval_results = compute_all_metrics(estimator.trajectory_history, gt_stream)
    ate = eval_results["ate"]
    rpe = eval_results["rpe"]
    vel = eval_results["velocity"]
    perf = eval_results["performance"]

    print(f"  * ATE RMSE:       {ate['ate_rmse_m']:.4f} m")
    print(f"  * RPE Trans RMSE: {rpe['rpe_trans_rmse_m']:.4f} m")
    print(f"  * Velocity RMSE:  {vel['vel_rmse_mps']:.4f} m/s")

    # 5. Save Artifacts
    seq_name = f"synthetic_{preset}"
    traj_dir = "results/trajectories"
    metrics_dir = "results/metrics"
    fig_dir = "results/figures"
    os.makedirs(traj_dir, exist_ok=True)
    os.makedirs(metrics_dir, exist_ok=True)
    os.makedirs(fig_dir, exist_ok=True)

    ori = eval_results["orientation"]

    metrics_json = os.path.join(metrics_dir, f"{seq_name}_metrics.json")
    with open(metrics_json, "w") as f:
        json.dump({
            "sequence": seq_name,
            "preset": preset,
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
        }, f, indent=4)

    plotter = VIOPlotter(output_dir=fig_dir)
    plotter.plot_3d_trajectory(eval_results, sequence_name=seq_name)
    plotter.plot_position_components(eval_results, sequence_name=seq_name)
    plotter.plot_errors_and_residuals(eval_results, sequence_name=seq_name)
    plotter.plot_velocities(eval_results, sequence_name=seq_name)
    plotter.plot_biases_and_features(estimator.trajectory_history, sequence_name=seq_name)
    plotter.plot_comprehensive_dashboard(eval_results, estimator.trajectory_history, sequence_name=seq_name)

    print(f"Saved metrics & figures for {seq_name}")


if __name__ == "__main__":
    presets = [
        ("orbit", False),
        ("multi_axis", False),
        ("hover", False),
        ("straight", False),
        ("stress", True)
    ]
    for p, d in presets:
        run_benchmark(p, d)

    # Run evaluate summary
    import pandas as pd
    import glob
    json_files = glob.glob(os.path.join("results", "metrics", "*_metrics.json"))
    records = []
    for jf in json_files:
        with open(jf, "r") as f:
            records.append(json.load(f))
    df = pd.DataFrame(records)
    print("\n" + "=" * 85)
    print("                 PHASE 1 VIO ESTIMATION BENCHMARK RESULTS TABLE")
    print("=" * 85)
    print(df.to_string(index=False))
    print("=" * 85)
    df.to_csv("results/metrics/benchmark_summary_table.csv", index=False)
