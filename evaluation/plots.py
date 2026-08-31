import os
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, Any, Optional


class VIOPlotter:
    """Publication-grade visualization dashboard generator."""

    def __init__(self, output_dir: str = "results/figures"):
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')

    def plot_summary_dashboard(self, eval_results: Dict[str, Any], sequence_name: str = "synthetic") -> str:
        """Generates comprehensive 6-panel trajectory and error analysis dashboard."""
        fig = plt.figure(figsize=(18, 11))

        t = eval_results["timestamps"]
        est_aligned = eval_results["est_positions_aligned"]
        gt_interp = eval_results["gt_positions_interp"]
        ate = eval_results["ate"]
        vel = eval_results["velocity"]
        ori = eval_results["orientation"]

        # 1. 3D Trajectory
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        ax1.plot(gt_interp[:, 0], gt_interp[:, 1], gt_interp[:, 2], 'k--', label='Ground Truth', linewidth=1.5)
        ax1.plot(est_aligned[:, 0], est_aligned[:, 1], est_aligned[:, 2], 'b-', label='MSCKF (Aligned)', linewidth=1.8)
        ax1.scatter(gt_interp[0, 0], gt_interp[0, 1], gt_interp[0, 2], c='g', s=60, label='Start')
        ax1.scatter(gt_interp[-1, 0], gt_interp[-1, 1], gt_interp[-1, 2], c='r', s=60, label='End')
        ax1.set_title(f"3D Trajectory Comparison - {sequence_name}", fontweight='bold')
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_zlabel("Z (m)")
        ax1.legend(loc='upper right', fontsize=8)

        # 2. XY Plane View
        ax2 = fig.add_subplot(2, 3, 2)
        ax2.plot(gt_interp[:, 0], gt_interp[:, 1], 'k--', label='Ground Truth', linewidth=1.5)
        ax2.plot(est_aligned[:, 0], est_aligned[:, 1], 'b-', label='MSCKF', linewidth=1.8)
        ax2.set_title("XY Top-Down View", fontweight='bold')
        ax2.set_xlabel("X (m)")
        ax2.set_ylabel("Y (m)")
        ax2.grid(True)
        ax2.legend(loc='upper right', fontsize=8)

        # 3. Position Error over Time
        ax3 = fig.add_subplot(2, 3, 3)
        ax3.plot(t, ate["errors"], 'r-', linewidth=1.5, label=f'ATE (RMSE: {ate["ate_rmse_m"]:.3f} m)')
        ax3.axhline(ate["ate_mean_m"], color='darkred', linestyle='--', label=f'Mean: {ate["ate_mean_m"]:.3f} m')
        ax3.set_title("Absolute Trajectory Error (ATE)", fontweight='bold')
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Error (m)")
        ax3.grid(True)
        ax3.legend(loc='upper right', fontsize=8)

        # 4. XYZ Position Decomposition
        ax4 = fig.add_subplot(2, 3, 4)
        ax4.plot(t, est_aligned[:, 0] - gt_interp[:, 0], 'r-', label='Error X')
        ax4.plot(t, est_aligned[:, 1] - gt_interp[:, 1], 'g-', label='Error Y')
        ax4.plot(t, est_aligned[:, 2] - gt_interp[:, 2], 'b-', label='Error Z')
        ax4.set_title("XYZ Position Tracking Residuals", fontweight='bold')
        ax4.set_xlabel("Time (s)")
        ax4.set_ylabel("Residual (m)")
        ax4.grid(True)
        ax4.legend(loc='upper right', fontsize=8)

        # 5. SO(3) Attitude Orientation Error
        ax5 = fig.add_subplot(2, 3, 5)
        ax5.plot(t, ori["ori_series_deg"], 'm-', linewidth=1.5, label=f'Orientation Error (RMSE: {ori["ori_rmse_deg"]:.2f}°)')
        ax5.axhline(ori["ori_final_deg"], color='purple', linestyle='--', label=f'Final: {ori["ori_final_deg"]:.2f}°')
        ax5.set_title("SO(3) Geodesic Attitude Error", fontweight='bold')
        ax5.set_xlabel("Time (s)")
        ax5.set_ylabel("Angle Error (deg)")
        ax5.grid(True)
        ax5.legend(loc='upper right', fontsize=8)

        # 6. Velocity Error
        ax6 = fig.add_subplot(2, 3, 6)
        ax6.plot(t, vel["errors"], 'teal', linewidth=1.5, label=f'Velocity Error (RMSE: {vel["vel_rmse_mps"]:.3f} m/s)')
        ax6.set_title("Velocity Estimation Error", fontweight='bold')
        ax6.set_xlabel("Time (s)")
        ax6.set_ylabel("Speed Error (m/s)")
        ax6.grid(True)
        ax6.legend(loc='upper right', fontsize=8)

        fig.tight_layout()
        save_path = os.path.join(self.output_dir, f"{sequence_name}_dashboard.png")
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        plt.close(fig)
        return save_path
