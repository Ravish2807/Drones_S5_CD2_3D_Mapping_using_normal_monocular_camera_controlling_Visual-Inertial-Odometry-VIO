import os
import sys
import yaml
import json
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.dataset_io.synthetic_generator import SyntheticDatasetGenerator
from src.estimator_wrapper.msckf_estimator import MSCKFEstimator
from src.evaluation.metrics import compute_all_metrics


def run_isolated_experiment(exp_id: str, exp_name: str, config_mod: dict, preset: str = "orbit") -> dict:
    print(f"\n[RUNNING TEST {exp_id}]: {exp_name}...", flush=True)

    with open("config/synthetic.yaml", "r") as f:
        config = yaml.safe_load(f)

    # Apply configuration modifications
    for k, v in config_mod.items():
        if isinstance(v, dict) and k in config:
            config[k].update(v)
        else:
            config[k] = v

    config["simulation"]["duration_sec"] = 15.0

    gen = SyntheticDatasetGenerator(config)
    inject_drop = config_mod.get("inject_dropout", False)
    ds = gen.generate_dataset(preset=preset, inject_dropout=inject_drop)

    imu_stream = ds["imu"]
    cam_stream = ds["camera"]
    gt_stream = ds["ground_truth"]

    estimator = MSCKFEstimator(config)
    gt0 = gt_stream[0]
    estimator.p_WB = gt0["position"].copy()
    estimator.v_WB = gt0["velocity"].copy()
    estimator.R_WB = gt0["R_WB"].copy()
    estimator.is_initialized = True

    # Inject time offset if specified (e.g. shift camera timestamps by dt_skew)
    time_skew = config_mod.get("time_skew_sec", 0.0)

    imu_idx = 0
    for cam in cam_stream:
        t_cam = cam["timestamp"] + time_skew
        while imu_idx < len(imu_stream) and imu_stream[imu_idx]["timestamp"] <= t_cam:
            s = imu_stream[imu_idx]
            estimator.process_imu(s["timestamp"], s["linear_accel"], s["angular_vel"])
            imu_idx += 1
        estimator.process_camera(t_cam, cam["image"])

    eval_res = compute_all_metrics(estimator.trajectory_history, gt_stream)
    ate = eval_res["ate"]
    rpe = eval_res["rpe"]
    vel = eval_res["velocity"]
    ori = eval_res["orientation"]
    perf = eval_res["performance"]

    record = {
        "experiment_id": exp_id,
        "test_condition": exp_name,
        "ate_rmse_m": ate["ate_rmse_m"],
        "ate_median_m": ate["ate_median_m"],
        "rpe_trans_m": rpe["rpe_trans_rmse_m"],
        "ori_rmse_deg": ori["ori_rmse_deg"],
        "ori_final_deg": ori["ori_final_deg"],
        "vel_rmse_mps": vel["vel_rmse_mps"],
        "latency_ms": perf["mean_proc_time_ms"]
    }
    print(f"  -> ATE RMSE: {ate['ate_rmse_m']:.4f} m | Ori RMSE: {ori['ori_rmse_deg']:.3f}° | Vel RMSE: {vel['vel_rmse_mps']:.3f} m/s", flush=True)
    return record


def main():
    print("=" * 80)
    print("      TASK GROUP I: SYSTEMATIC VIO NOISE, BIAS & TIMING SENSITIVITY STUDY")
    print("=" * 80)

    experiments = [
        ("I0 (Ideal B0)", "Zero noise, zero bias, perfect sensors", {
            "imu": {"noise_accel": 0.0, "noise_gyro": 0.0, "random_walk_accel": 0.0, "random_walk_gyro": 0.0},
            "time_skew_sec": 0.0
        }),
        ("I1 (Noise Only)", "IMU white noise only (no bias)", {
            "imu": {"noise_accel": 0.005, "noise_gyro": 0.001, "random_walk_accel": 0.0, "random_walk_gyro": 0.0}
        }),
        ("I2 (Gyro Bias)", "Constant Gyro bias (bg = 0.005 rad/s)", {
            "imu": {"noise_accel": 0.0, "noise_gyro": 0.0, "random_walk_accel": 0.0, "random_walk_gyro": 0.0001}
        }),
        ("I3 (Accel Bias)", "Constant Accel bias (ba = 0.05 m/s²)", {
            "imu": {"noise_accel": 0.0, "noise_gyro": 0.0, "random_walk_accel": 0.001, "random_walk_gyro": 0.0}
        }),
        ("I4 (Dropout)", "50% visual feature dropouts", {
            "inject_dropout": True
        }),
        ("I5 (Time Offset)", "15 ms temporal unsynchronized skew", {
            "time_skew_sec": 0.015
        }),
        ("I6 (Realistic)", "Combined realistic sensor noise & bias random walk", {
            "imu": {"noise_accel": 0.005, "noise_gyro": 0.001, "random_walk_accel": 0.0001, "random_walk_gyro": 0.00001}
        })
    ]

    records = []
    for exp_id, name, mod in experiments:
        rec = run_isolated_experiment(exp_id, name, mod, preset="orbit")
        records.append(rec)

    df = pd.DataFrame(records)
    print("\n" + "=" * 90)
    print("                     SENSITIVITY STUDY RESULTS MATRIX")
    print("=" * 90)
    print(df.to_string(index=False))
    print("=" * 90)

    out_csv = "results/metrics/sensitivity_study_table.csv"
    df.to_csv(out_csv, index=False)
    print(f"Exported Sensitivity Study Matrix to: {out_csv}")


if __name__ == "__main__":
    main()
