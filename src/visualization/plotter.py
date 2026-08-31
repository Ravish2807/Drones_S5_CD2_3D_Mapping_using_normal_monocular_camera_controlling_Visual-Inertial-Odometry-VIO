import os
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, Any, Optional


class VIOPlotter:
    """
    Visualization engine for the VIO state estimation pipeline.
    Generates publication-quality 3D trajectory plots, temporal error curves,
    state dynamics comparisons, and sensor bias evolution.
    """

    def __init__(self, output_dir: str = "results/figures"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        # Use clean modern plotting style
        plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")

    def plot_3d_trajectory(self, eval_results: Dict[str, Any], sequence_name: str = "trajectory") -> str:
        """Plots 3D spatial ground truth vs. estimated trajectory."""
        est_pos = eval_results["est_positions_aligned"]
        gt_pos = eval_results["gt_positions"]

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        ax.plot(gt_pos[:, 0], gt_pos[:, 1], gt_pos[:, 2], 'k--', label="Ground Truth", linewidth=2.0)
        ax.plot(est_pos[:, 0], est_pos[:, 1], est_pos[:, 2], 'b-', label="MSCKF VIO (Aligned)", linewidth=2.0)

        # Mark start and end points
        ax.scatter(gt_pos[0, 0], gt_pos[0, 1], gt_pos[0, 2], color='green', s=60, label="Start")
        ax.scatter(gt_pos[-1, 0], gt_pos[-1, 1], gt_pos[-1, 2], color='red', s=60, label="End (GT)")
        ax.scatter(est_pos[-1, 0], est_pos[-1, 1], est_pos[-1, 2], color='orange', s=60, label="End (VIO)")

        ax.set_title(f"3D Trajectory Estimation - {sequence_name.upper()}\nATE RMSE: {eval_results['ate']['ate_rmse_m']:.3f} m", fontsize=13, fontweight="bold")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.legend(loc="upper right")
        fig.tight_layout()

        out_path = os.path.join(self.output_dir, f"{sequence_name}_3d_trajectory.png")
        plt.savefig(out_path, dpi=200)
        plt.close(fig)
        return out_path

    def plot_position_components(self, eval_results: Dict[str, Any], sequence_name: str = "trajectory") -> str:
        """Plots X(t), Y(t), Z(t) estimated vs ground truth."""
        t = eval_results["timestamps"]
        est_pos = eval_results["est_positions_aligned"]
        gt_pos = eval_results["gt_positions"]

        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axis_names = ["X (m)", "Y (m)", "Z (m)"]

        for i in range(3):
            axs[i].plot(t, gt_pos[:, i], 'k--', label="Ground Truth", linewidth=1.8)
            axs[i].plot(t, est_pos[:, i], 'b-', label="MSCKF VIO", linewidth=1.8)
            axs[i].set_ylabel(axis_names[i], fontsize=11)
            axs[i].legend(loc="upper right")
            axs[i].grid(True)

        axs[2].set_xlabel("Time (s)", fontsize=11)
        fig.suptitle(f"Position Evolution Over Time - {sequence_name.upper()}", fontsize=13, fontweight="bold")
        fig.tight_layout()

        out_path = os.path.join(self.output_dir, f"{sequence_name}_position_xyz.png")
        plt.savefig(out_path, dpi=200)
        plt.close(fig)
        return out_path

    def plot_errors_and_residuals(self, eval_results: Dict[str, Any], sequence_name: str = "trajectory") -> str:
        """Plots position error vs time and error histogram."""
        t = eval_results["timestamps"]
        err = eval_results["ate"]["error_series"]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))

        ax1.plot(t, err, 'r-', linewidth=1.5, label="Position Error ||p_gt - p_est||")
        ax1.axhline(eval_results["ate"]["ate_rmse_m"], color="black", linestyle="--",
                    label=f"ATE RMSE ({eval_results['ate']['ate_rmse_m']:.3f} m)")
        ax1.set_ylabel("Error (m)", fontsize=11)
        ax1.set_xlabel("Time (s)", fontsize=11)
        ax1.set_title("Absolute Trajectory Error Over Time", fontsize=12, fontweight="bold")
        ax1.legend(loc="upper right")
        ax1.grid(True)

        # Histogram
        ax2.hist(err, bins=25, color="skyblue", edgecolor="black", alpha=0.7)
        ax2.axvline(eval_results["ate"]["ate_mean_m"], color="red", linestyle="dashed",
                    linewidth=2, label=f"Mean Error: {eval_results['ate']['ate_mean_m']:.3f} m")
        ax2.axvline(eval_results["ate"]["ate_median_m"], color="green", linestyle="dotted",
                    linewidth=2, label=f"Median Error: {eval_results['ate']['ate_median_m']:.3f} m")
        ax2.set_xlabel("Position Error (m)", fontsize=11)
        ax2.set_ylabel("Count", fontsize=11)
        ax2.set_title("Error Distribution", fontsize=12, fontweight="bold")
        ax2.legend(loc="upper right")
        ax2.grid(True)

        fig.tight_layout()
        out_path = os.path.join(self.output_dir, f"{sequence_name}_error_analysis.png")
        plt.savefig(out_path, dpi=200)
        plt.close(fig)
        return out_path

    def plot_velocities(self, eval_results: Dict[str, Any], sequence_name: str = "trajectory") -> str:
        """Plots estimated vs ground truth velocities."""
        t = eval_results["timestamps"]
        est_vel = eval_results["est_velocities"]
        gt_vel = eval_results["gt_velocities"]

        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        vel_names = ["Vx (m/s)", "Vy (m/s)", "Vz (m/s)"]

        for i in range(3):
            axs[i].plot(t, gt_vel[:, i], 'k--', label="GT Velocity", linewidth=1.8)
            axs[i].plot(t, est_vel[:, i], 'g-', label="Estimated Velocity", linewidth=1.8)
            axs[i].set_ylabel(vel_names[i], fontsize=11)
            axs[i].legend(loc="upper right")
            axs[i].grid(True)

        axs[2].set_xlabel("Time (s)", fontsize=11)
        fig.suptitle(f"Velocity Profile - {sequence_name.upper()} (RMSE: {eval_results['velocity']['vel_rmse_mps']:.3f} m/s)",
                     fontsize=13, fontweight="bold")
        fig.tight_layout()

        out_path = os.path.join(self.output_dir, f"{sequence_name}_velocity.png")
        plt.savefig(out_path, dpi=200)
        plt.close(fig)
        return out_path

    def plot_biases_and_features(self, est_records: list, sequence_name: str = "trajectory") -> str:
        """Plots accelerometer/gyroscope bias estimates and tracked feature counts."""
        t = [r["timestamp"] for r in est_records]
        ba = np.array([r["bias_accel"] for r in est_records])
        bg = np.array([r["bias_gyro"] for r in est_records])
        feat_active = [r.get("active_features", 0) for r in est_records]
        feat_updated = [r.get("updated_features", 0) for r in est_records]

        fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

        # Accel Bias
        axs[0].plot(t, ba[:, 0], 'r-', label="b_ax")
        axs[0].plot(t, ba[:, 1], 'g-', label="b_ay")
        axs[0].plot(t, ba[:, 2], 'b-', label="b_az")
        axs[0].set_ylabel("Accel Bias (m/s²)", fontsize=11)
        axs[0].set_title("Estimated Accelerometer Bias Over Time", fontsize=12, fontweight="bold")
        axs[0].legend(loc="upper right")
        axs[0].grid(True)

        # Gyro Bias
        axs[1].plot(t, bg[:, 0], 'r--', label="b_gx")
        axs[1].plot(t, bg[:, 1], 'g--', label="b_gy")
        axs[1].plot(t, bg[:, 2], 'b--', label="b_gz")
        axs[1].set_ylabel("Gyro Bias (rad/s)", fontsize=11)
        axs[1].set_title("Estimated Gyroscope Bias Over Time", fontsize=12, fontweight="bold")
        axs[1].legend(loc="upper right")
        axs[1].grid(True)

        # Feature Counts
        axs[2].plot(t, feat_active, 'purple', label="Tracked Features")
        axs[2].plot(t, feat_updated, 'orange', label="MSCKF Updated Features")
        axs[2].set_ylabel("Count", fontsize=11)
        axs[2].set_xlabel("Time (s)", fontsize=11)
        axs[2].set_title("Visual Feature Tracking Quality", fontsize=12, fontweight="bold")
        axs[2].legend(loc="upper right")
        axs[2].grid(True)

        fig.tight_layout()
        out_path = os.path.join(self.output_dir, f"{sequence_name}_biases_features.png")
        plt.savefig(out_path, dpi=200)
        plt.close(fig)
        return out_path

    def plot_comprehensive_dashboard(self, eval_results: Dict[str, Any], est_records: list, sequence_name: str = "summary") -> str:
        """Generates a complete 6-panel summary dashboard for the technical report."""
        t = eval_results["timestamps"]
        est_pos = eval_results["est_positions_aligned"]
        gt_pos = eval_results["gt_positions"]
        err = eval_results["ate"]["error_series"]
        proc_times_ms = [r.get("proc_time_sec", 0.0) * 1000.0 for r in est_records]

        fig = plt.figure(figsize=(16, 11))

        # Panel 1: 3D Trajectory
        ax1 = fig.add_subplot(2, 3, 1, projection="3d")
        ax1.plot(gt_pos[:, 0], gt_pos[:, 1], gt_pos[:, 2], 'k--', label="Ground Truth", linewidth=1.5)
        ax1.plot(est_pos[:, 0], est_pos[:, 1], est_pos[:, 2], 'b-', label="MSCKF VIO", linewidth=1.5)
        ax1.set_title("3D Trajectory", fontsize=11, fontweight="bold")
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_zlabel("Z (m)")
        ax1.legend(loc="upper right", fontsize=8)

        # Panel 2: 2D XY Top-Down View
        ax2 = fig.add_subplot(2, 3, 2)
        ax2.plot(gt_pos[:, 0], gt_pos[:, 1], 'k--', label="GT", linewidth=1.5)
        ax2.plot(est_pos[:, 0], est_pos[:, 1], 'b-', label="VIO", linewidth=1.5)
        ax2.set_title("Top-Down (X-Y) Profile", fontsize=11, fontweight="bold")
        ax2.set_xlabel("X (m)")
        ax2.set_ylabel("Y (m)")
        ax2.legend(loc="upper right", fontsize=8)
        ax2.grid(True)

        # Panel 3: Altitude Z(t)
        ax3 = fig.add_subplot(2, 3, 3)
        ax3.plot(t, gt_pos[:, 2], 'k--', label="GT Altitude", linewidth=1.5)
        ax3.plot(t, est_pos[:, 2], 'b-', label="VIO Altitude", linewidth=1.5)
        ax3.set_title("Altitude Z(t)", fontsize=11, fontweight="bold")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Z (m)")
        ax3.legend(loc="upper right", fontsize=8)
        ax3.grid(True)

        # Panel 4: ATE Error vs Time
        ax4 = fig.add_subplot(2, 3, 4)
        ax4.plot(t, err, 'r-', linewidth=1.5, label="Position Error")
        ax4.axhline(eval_results["ate"]["ate_rmse_m"], color="black", linestyle="--",
                    label=f"RMSE: {eval_results['ate']['ate_rmse_m']:.3f} m")
        ax4.set_title("Position Error Over Time", fontsize=11, fontweight="bold")
        ax4.set_xlabel("Time (s)")
        ax4.set_ylabel("Error (m)")
        ax4.legend(loc="upper right", fontsize=8)
        ax4.grid(True)

        # Panel 5: Feature Tracking
        ax5 = fig.add_subplot(2, 3, 5)
        feat_active = [r.get("active_features", 0) for r in est_records]
        ax5.plot(t[:len(feat_active)], feat_active, 'purple', label="Tracked Features")
        ax5.set_title("Tracked Features vs Time", fontsize=11, fontweight="bold")
        ax5.set_xlabel("Time (s)")
        ax5.set_ylabel("Feature Count")
        ax5.legend(loc="upper right", fontsize=8)
        ax5.grid(True)

        # Panel 6: Runtime & Processing Latency
        ax6 = fig.add_subplot(2, 3, 6)
        ax6.plot(proc_times_ms, 'teal', label="Frame Processing (ms)")
        ax6.axhline(50.0, color="orange", linestyle="--", label="20 Hz Real-Time Threshold (50ms)")
        ax6.set_title(f"Processing Latency (Mean: {np.mean(proc_times_ms):.1f} ms)", fontsize=11, fontweight="bold")
        ax6.set_xlabel("Frame Index")
        ax6.set_ylabel("Latency (ms)")
        ax6.legend(loc="upper right", fontsize=8)
        ax6.grid(True)

        fig.suptitle(f"MSCKF VIO Evaluation Dashboard - {sequence_name.upper()} | ATE RMSE: {eval_results['ate']['ate_rmse_m']:.3f} m",
                     fontsize=14, fontweight="bold")
        fig.tight_layout()

        out_path = os.path.join(self.output_dir, f"{sequence_name}_comprehensive_dashboard.png")
        plt.savefig(out_path, dpi=200)
        plt.close(fig)
        return out_path
