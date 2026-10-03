import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from typing import Dict, Any, Tuple, Optional
from scipy.spatial.transform import Rotation

from .camera_pose import CameraTrajectory, CameraPose
from evaluation.trajectory_alignment import align_trajectories_se3, interpolate_trajectory
from core.math_utils import log_so3


def compute_camera_trajectory_metrics(
    est_traj: CameraTrajectory,
    gt_traj: CameraTrajectory,
    alignment_mode: str = "se3",
    delta_sec: float = 1.0
) -> Dict[str, Any]:
    """
    Step 2.8: Computes standardized camera trajectory metrics:
    - Camera Position ATE (RMSE, Mean, Median, Max, Std)
    - Camera Relative Motion Drift (RPE translation and rotation)
    - Camera Orientation Error (e_R in degrees)
    - Camera Linear Velocity Error (e_v in m/s)
    """
    if len(est_traj) < 2 or len(gt_traj) < 2:
        raise ValueError("Trajectories must have at least 2 poses for evaluation.")

    est_times = est_traj.timestamps
    gt_times = gt_traj.timestamps

    # Restrict to overlapping time window
    t_start = max(est_times[0], gt_times[0])
    t_end = min(est_times[-1], gt_times[-1])

    valid_mask = (est_times >= t_start) & (est_times <= t_end)
    eval_times = est_times[valid_mask]
    est_positions = est_traj.positions[valid_mask]
    est_rotations = est_traj.rotations[valid_mask]

    # Interpolate Ground Truth at estimator timestamps
    gt_positions = interpolate_trajectory(eval_times, gt_times, gt_traj.positions)

    # Interpolate GT rotations via SLERP
    gt_rot_objs = Rotation.from_matrix(gt_traj.rotations)
    gt_slerp = gt_rot_objs  # Can interpolate directly or map
    gt_interp_rot = []
    for t in eval_times:
        gt_pose = gt_traj.interpolate_pose(t)
        if gt_pose is not None:
            gt_interp_rot.append(gt_pose.R_WC)
        else:
            gt_interp_rot.append(np.eye(3))
    gt_rotations = np.array(gt_interp_rot)

    # 1. SE(3) Rigid Trajectory Alignment
    with_scale = (alignment_mode.lower() == "sim3")
    R_align, t_align, scale_align, aligned_est_pos = align_trajectories_se3(
        est_positions, gt_positions, with_scale=with_scale
    )

    # 2. Camera ATE (Absolute Trajectory Error)
    ate_errors = np.linalg.norm(aligned_est_pos - gt_positions, axis=1)
    ate_rmse = float(np.sqrt(np.mean(ate_errors ** 2)))
    ate_mean = float(np.mean(ate_errors))
    ate_median = float(np.median(ate_errors))
    ate_max = float(np.max(ate_errors))
    ate_std = float(np.std(ate_errors))

    # 3. Camera RPE (Relative Pose Error over delta_sec)
    rpe_trans_errors = []
    rpe_rot_errors_deg = []
    for i in range(len(eval_times)):
        t_target = eval_times[i] + delta_sec
        if t_target > eval_times[-1]:
            break
        j = int(np.argmin(np.abs(eval_times - t_target)))
        if j == i:
            continue

        # Relative delta GT
        delta_p_gt = gt_positions[j] - gt_positions[i]
        # Relative delta Est (aligned)
        delta_p_est = aligned_est_pos[j] - aligned_est_pos[i]

        rpe_trans = np.linalg.norm(delta_p_est - delta_p_gt)
        rpe_trans_errors.append(rpe_trans)

        # Relative rotation
        R_rel_gt = gt_rotations[i].T @ gt_rotations[j]
        R_rel_est = est_rotations[i].T @ est_rotations[j]
        R_err = R_rel_gt.T @ R_rel_est
        ang_err = np.linalg.norm(log_so3(R_err))
        rpe_rot_errors_deg.append(np.degrees(ang_err))

    rpe_trans_arr = np.array(rpe_trans_errors) if len(rpe_trans_errors) > 0 else np.array([0.0])
    rpe_rot_arr = np.array(rpe_rot_errors_deg) if len(rpe_rot_errors_deg) > 0 else np.array([0.0])

    rpe_trans_rmse = float(np.sqrt(np.mean(rpe_trans_arr ** 2)))
    rpe_trans_mean = float(np.mean(rpe_trans_arr))
    rpe_rot_rmse_deg = float(np.sqrt(np.mean(rpe_rot_arr ** 2)))

    # 4. Camera Orientation Errors (geodesic rotation error on SO(3))
    ori_errors_deg = []
    for i in range(len(eval_times)):
        # Apply alignment rotation to estimated camera orientation
        R_est_aligned = R_align @ est_rotations[i]
        R_err = gt_rotations[i].T @ R_est_aligned
        ang_rad = np.linalg.norm(log_so3(R_err))
        ori_errors_deg.append(np.degrees(ang_rad))

    ori_errors_arr = np.array(ori_errors_deg)
    ori_rmse_deg = float(np.sqrt(np.mean(ori_errors_arr ** 2)))
    ori_mean_deg = float(np.mean(ori_errors_arr))
    ori_final_deg = float(ori_errors_arr[-1]) if len(ori_errors_arr) > 0 else 0.0

    # 5. Camera Velocity Errors (numerical velocity difference)
    dt_arr = np.diff(eval_times)
    vel_rmse_mps = 0.0
    if len(dt_arr) > 0 and np.all(dt_arr > 0):
        v_est = np.diff(aligned_est_pos, axis=0) / dt_arr[:, None]
        v_gt = np.diff(gt_positions, axis=0) / dt_arr[:, None]
        v_errors = np.linalg.norm(v_est - v_gt, axis=1)
        vel_rmse_mps = float(np.sqrt(np.mean(v_errors ** 2)))

    return {
        "ate": {
            "rmse_m": ate_rmse,
            "mean_m": ate_mean,
            "median_m": ate_median,
            "max_m": ate_max,
            "std_m": ate_std,
            "errors": ate_errors
        },
        "rpe": {
            "trans_rmse_m": rpe_trans_rmse,
            "trans_mean_m": rpe_trans_mean,
            "rot_rmse_deg": rpe_rot_rmse_deg
        },
        "orientation": {
            "rmse_deg": ori_rmse_deg,
            "mean_deg": ori_mean_deg,
            "final_deg": ori_final_deg,
            "errors_deg": ori_errors_arr
        },
        "velocity": {
            "rmse_mps": vel_rmse_mps
        },
        "alignment": {
            "R": R_align,
            "t": t_align,
            "scale": scale_align
        },
        "eval_times": eval_times,
        "aligned_est_pos": aligned_est_pos,
        "gt_positions": gt_positions,
        "est_rotations": est_rotations,
        "gt_rotations": gt_rotations
    }


def plot_camera_trajectory_comparison(
    metrics: Dict[str, Any],
    output_dir: str,
    sequence_name: str
) -> Tuple[str, str]:
    """
    Step 2.7: Generates publication-ready visualizations:
    1. 3D Camera Trajectory: Ground Truth vs VIO Estimated.
    2. Comprehensive Error Analysis Dashboard (XYZ errors, ATE, Orientation error).
    """
    os.makedirs(output_dir, exist_ok=True)
    times = metrics["eval_times"]
    est_pos = metrics["aligned_est_pos"]
    gt_pos = metrics["gt_positions"]
    ate = metrics["ate"]
    ori = metrics["orientation"]

    # -------------------------------------------------------------
    # Figure 1: 3D Spatial Trajectory Plot
    # -------------------------------------------------------------
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(gt_pos[:, 0], gt_pos[:, 1], gt_pos[:, 2],
            label="Ground Truth Camera $T_{WC}^{GT}$", color="#1f77b4", linewidth=2.5, alpha=0.9)
    ax.plot(est_pos[:, 0], est_pos[:, 1], est_pos[:, 2],
            label=f"VIO Estimated Camera $T_{{WC}}^{{Est}}$ (ATE={ate['rmse_m']:.3f}m)",
            color="#ff7f0e", linestyle="--", linewidth=2.0)

    # Start and End markers
    ax.scatter([gt_pos[0, 0]], [gt_pos[0, 1]], [gt_pos[0, 2]], color="green", s=80, marker="o", label="Start (t=0)")
    ax.scatter([gt_pos[-1, 0]], [gt_pos[-1, 1]], [gt_pos[-1, 2]], color="red", s=90, marker="X", label="End")

    ax.set_title(f"3D Camera Trajectory ($T_{{WC}}$): {sequence_name.upper()}\nATE RMSE = {ate['rmse_m']:.4f} m | RPE = {metrics['rpe']['trans_rmse_m']:.4f} m", fontsize=12, fontweight="bold")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend(loc="upper right", framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.6)

    fig_3d_path = os.path.join(output_dir, f"{sequence_name}_camera_trajectory_3d.png")
    plt.tight_layout()
    plt.savefig(fig_3d_path, dpi=300)
    plt.close(fig)

    # -------------------------------------------------------------
    # Figure 2: Detailed Error Analysis Dashboard
    # -------------------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

    # Subplot A: XYZ Coordinates over Time
    axes[0].plot(times, gt_pos[:, 0], 'r-', label="X GT", alpha=0.8)
    axes[0].plot(times, est_pos[:, 0], 'r--', label="X Est", alpha=0.8)
    axes[0].plot(times, gt_pos[:, 1], 'g-', label="Y GT", alpha=0.8)
    axes[0].plot(times, est_pos[:, 1], 'g--', label="Y Est", alpha=0.8)
    axes[0].plot(times, gt_pos[:, 2], 'b-', label="Z GT", alpha=0.8)
    axes[0].plot(times, est_pos[:, 2], 'b--', label="Z Est", alpha=0.8)
    axes[0].set_ylabel("Position (m)")
    axes[0].set_title(f"Camera Trajectory Tracking Breakdown — {sequence_name.upper()}", fontweight="bold")
    axes[0].legend(loc="upper right", ncol=3, fontsize=9)
    axes[0].grid(True, linestyle=":", alpha=0.6)

    # Subplot B: 3D Euclidean Position Error (ATE)
    axes[1].plot(times, ate["errors"], color="#d62728", linewidth=1.5, label=r"3D Position Error $\|p_{WC}^{Est} - p_{WC}^{GT}\|$")
    axes[1].axhline(ate["rmse_m"], color="black", linestyle="--", label=f"ATE RMSE ({ate['rmse_m']:.4f} m)")
    axes[1].set_ylabel("ATE Error (m)")
    axes[1].legend(loc="upper right", fontsize=9)
    axes[1].grid(True, linestyle=":", alpha=0.6)

    # Subplot C: Camera Orientation Error (Degrees)
    axes[2].plot(times, ori["errors_deg"], color="#9467bd", linewidth=1.5, label="Orientation Geodesic Error ($e_R$)")
    axes[2].axhline(ori["rmse_deg"], color="black", linestyle="--", label=f"Ori RMSE ({ori['rmse_deg']:.2f}°)")
    axes[2].set_ylabel("Ori Error (deg)")
    axes[2].set_xlabel("Time (seconds)")
    axes[2].legend(loc="upper right", fontsize=9)
    axes[2].grid(True, linestyle=":", alpha=0.6)

    fig_dash_path = os.path.join(output_dir, f"{sequence_name}_camera_error_dashboard.png")
    plt.tight_layout()
    plt.savefig(fig_dash_path, dpi=300)
    plt.close(fig)

    return fig_3d_path, fig_dash_path
