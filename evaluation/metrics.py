import numpy as np
from typing import Dict, Any, List, Tuple
from scipy.spatial.transform import Rotation
from .trajectory_alignment import align_trajectories_se3, interpolate_trajectory
from core.math_utils import quat_to_rot


def compute_ate(aligned_est_pos: np.ndarray, gt_pos: np.ndarray) -> Dict[str, Any]:
    """Computes Absolute Trajectory Error (ATE) statistics."""
    errors = np.linalg.norm(aligned_est_pos - gt_pos, axis=1)
    rmse = float(np.sqrt(np.mean(errors ** 2)))
    mean_err = float(np.mean(errors))
    median_err = float(np.median(errors))
    std_err = float(np.std(errors))
    max_err = float(np.max(errors))

    return {
        "ate_rmse_m": rmse,
        "ate_mean_m": mean_err,
        "ate_median_m": median_err,
        "ate_std_m": std_err,
        "ate_max_m": max_err,
        "errors": errors
    }


def compute_rpe(aligned_est_pos: np.ndarray, gt_pos: np.ndarray, delta_step: int = 10) -> Dict[str, Any]:
    """Computes Relative Pose Error (RPE) translational drift statistics."""
    if len(aligned_est_pos) <= delta_step:
        return {"rpe_trans_rmse_m": 0.0, "rpe_trans_mean_m": 0.0, "rpe_trans_max_m": 0.0, "errors": np.array([])}

    trans_errors = []
    for i in range(len(aligned_est_pos) - delta_step):
        d_est = aligned_est_pos[i + delta_step] - aligned_est_pos[i]
        d_gt = gt_pos[i + delta_step] - gt_pos[i]
        trans_errors.append(np.linalg.norm(d_est - d_gt))

    trans_errors = np.array(trans_errors)
    return {
        "rpe_trans_rmse_m": float(np.sqrt(np.mean(trans_errors ** 2))),
        "rpe_trans_mean_m": float(np.mean(trans_errors)),
        "rpe_trans_max_m": float(np.max(trans_errors)),
        "errors": trans_errors
    }


def compute_velocity_rmse(est_vel: np.ndarray, gt_vel: np.ndarray) -> Dict[str, Any]:
    """Computes Velocity Estimation Error metrics."""
    vel_err = np.linalg.norm(est_vel - gt_vel, axis=1)
    return {
        "vel_rmse_mps": float(np.sqrt(np.mean(vel_err ** 2))),
        "vel_mean_mps": float(np.mean(vel_err)),
        "vel_max_mps": float(np.max(vel_err)),
        "errors": vel_err
    }


def compute_orientation_error(est_rotations: List[np.ndarray], gt_rotations: List[np.ndarray]) -> Dict[str, Any]:
    """Computes SO(3) angular attitude error in degrees."""
    angles_deg = []
    euler_errors_deg = []

    for R_est, R_gt in zip(est_rotations, gt_rotations):
        R_err = R_gt.T @ R_est
        cos_theta = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
        angle_rad = np.arccos(cos_theta)
        angles_deg.append(np.degrees(angle_rad))

        r_diff = Rotation.from_matrix(R_err)
        euler_errors_deg.append(r_diff.as_euler('xyz', degrees=True))

    angles_deg = np.array(angles_deg)
    euler_errors_deg = np.array(euler_errors_deg)

    return {
        "ori_rmse_deg": float(np.sqrt(np.mean(angles_deg ** 2))),
        "ori_median_deg": float(np.median(angles_deg)),
        "ori_max_deg": float(np.max(angles_deg)),
        "ori_final_deg": float(angles_deg[-1]) if len(angles_deg) > 0 else 0.0,
        "roll_rmse_deg": float(np.sqrt(np.mean(euler_errors_deg[:, 0] ** 2))),
        "pitch_rmse_deg": float(np.sqrt(np.mean(euler_errors_deg[:, 1] ** 2))),
        "yaw_rmse_deg": float(np.sqrt(np.mean(euler_errors_deg[:, 2] ** 2))),
        "ori_series_deg": angles_deg
    }


def compute_all_metrics(est_records: List[Any], gt_records: List[Any]) -> Dict[str, Any]:
    """Master evaluator aggregating ATE, RPE, Orientation, Velocity and Latency."""
    if len(est_records) == 0 or len(gt_records) == 0:
        raise ValueError("Empty state records provided for evaluation.")

    # Standardize dictionary / dataclass records
    def to_dict(r):
        return r.to_dict() if hasattr(r, "to_dict") else r

    est_dicts = [to_dict(r) for r in est_records]
    gt_dicts = [to_dict(r) for r in gt_records]

    est_times = np.array([r["timestamp"] for r in est_dicts])
    gt_times = np.array([r["timestamp"] for r in gt_dicts])

    est_pos = np.array([r["position"] for r in est_dicts])
    gt_pos_all = np.array([r["position"] for r in gt_dicts])

    est_vel = np.array([r["velocity"] for r in est_dicts])
    gt_vel_all = np.array([r["velocity"] for r in gt_dicts])

    t_min = max(est_times[0], gt_times[0])
    t_max = min(est_times[-1], gt_times[-1])
    valid_mask = (est_times >= t_min) & (est_times <= t_max)

    est_times = est_times[valid_mask]
    est_pos = est_pos[valid_mask]
    est_vel = est_vel[valid_mask]

    gt_pos_interp = interpolate_trajectory(est_times, gt_times, gt_pos_all)
    gt_vel_interp = interpolate_trajectory(est_times, gt_times, gt_vel_all)

    # 1. SE(3) Alignment
    R_align, t_align, scale, aligned_est_pos = align_trajectories_se3(est_pos, gt_pos_interp, with_scale=False)

    # 2. ATE & RPE & Velocity
    ate_metrics = compute_ate(aligned_est_pos, gt_pos_interp)
    rpe_metrics = compute_rpe(aligned_est_pos, gt_pos_interp, delta_step=10)
    vel_metrics = compute_velocity_rmse(est_vel, gt_vel_interp)

    # 3. Orientations with SE(3) alignment rotation applied
    est_rotations_raw = []
    for r in est_dicts:
        if "orientation_R" in r:
            est_rotations_raw.append(r["orientation_R"])
        elif "orientation_q" in r:
            est_rotations_raw.append(quat_to_rot(r["orientation_q"]))
        else:
            est_rotations_raw.append(np.eye(3))

    est_rotations_aligned = [R_align @ R for R in est_rotations_raw]

    gt_rotations = []
    for t_query in est_times:
        idx = np.argmin(np.abs(gt_times - t_query))
        if "R_WB" in gt_dicts[idx]:
            gt_rotations.append(gt_dicts[idx]["R_WB"])
        elif "orientation_q" in gt_dicts[idx]:
            gt_rotations.append(quat_to_rot(gt_dicts[idx]["orientation_q"]))
        else:
            gt_rotations.append(np.eye(3))

    ori_metrics = compute_orientation_error(est_rotations_aligned[:len(est_times)], gt_rotations)

    # 4. Latency
    proc_times = [r.get("latency_sec", r.get("proc_time_sec", 0.0)) for r in est_dicts]
    mean_proc_time = float(np.mean(proc_times)) if len(proc_times) > 0 else 0.0

    return {
        "timestamps": est_times,
        "est_positions_raw": est_pos,
        "est_positions_aligned": aligned_est_pos,
        "gt_positions_interp": gt_pos_interp,
        "alignment_R": R_align,
        "alignment_t": t_align,
        "ate": ate_metrics,
        "rpe": rpe_metrics,
        "velocity": vel_metrics,
        "orientation": ori_metrics,
        "performance": {
            "mean_proc_time_ms": mean_proc_time * 1000.0,
            "effective_fps": (1.0 / mean_proc_time) if mean_proc_time > 0 else 0.0
        }
    }
