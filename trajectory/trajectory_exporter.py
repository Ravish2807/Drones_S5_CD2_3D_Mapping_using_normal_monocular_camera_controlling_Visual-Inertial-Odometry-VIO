import os
import csv
import numpy as np
from typing import List, Dict, Any, Optional
from scipy.spatial.transform import Rotation

from .camera_pose import CameraPose, CameraTrajectory


class TrajectoryExporter:
    """
    Standardized Trajectory Exporter for Phase 2.
    Supports:
    - Standard TUM Camera Trajectory format: 'timestamp tx ty tz qx qy qz qw'
    - Standard CSV format: 'timestamp,tx,ty,tz,qx,qy,qz,qw'
    - Full State CSV (Body + Camera + Velocity + Biases)
    - Timestamp Synchronization Validation CSV
    """

    @staticmethod
    def export_csv(trajectory: CameraTrajectory, filepath: str):
        """
        Exports standardized CSV camera trajectory.
        Format:
        timestamp,tx,ty,tz,qx,qy,qz,qw
        """
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"])
            for p in trajectory.poses:
                writer.writerow(p.to_csv_row())

    @staticmethod
    def export_tum(trajectory: CameraTrajectory, filepath: str):
        """
        Exports standard TUM format camera trajectory:
        '# timestamp tx ty tz qx qy qz qw'
        """
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        with open(filepath, "w") as f:
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            for p in trajectory.poses:
                f.write(p.to_tum_line() + "\n")

    @staticmethod
    def export_alignment_log(alignment_report: Dict[str, Any], filepath: str):
        """
        Step 2.9: Exports timestamp alignment log between image frames and camera poses.
        """
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["frame_idx", "image_time", "pose_time", "delta_t_sec", "delta_t_ms", "aligned"])
            for r in alignment_report.get("records", []):
                writer.writerow([
                    r["frame_idx"],
                    f"{r['image_time']:.6f}",
                    f"{r['pose_time']:.6f}",
                    f"{r['delta_t_sec']:.8f}",
                    f"{r['delta_t_ms']:.5f}",
                    r["aligned"]
                ])

    @staticmethod
    def load_csv(filepath: str) -> CameraTrajectory:
        """Loads a CameraTrajectory from CSV."""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File not found: {filepath}")

        poses = []
        with open(filepath, "r") as f:
            reader = csv.reader(f)
            header = next(reader, None)
            for row in reader:
                if len(row) >= 8:
                    t = float(row[0])
                    tx, ty, tz = float(row[1]), float(row[2]), float(row[3])
                    qx, qy, qz, qw = float(row[4]), float(row[5]), float(row[6]), float(row[7])
                    R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
                    poses.append(CameraPose(timestamp=t, p_WC=np.array([tx, ty, tz]), R_WC=R))

        return CameraTrajectory(poses, name=os.path.basename(filepath))

    @staticmethod
    def load_tum(filepath: str) -> CameraTrajectory:
        """Loads a CameraTrajectory from standard TUM text file."""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File not found: {filepath}")

        poses = []
        with open(filepath, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) >= 8:
                    t = float(parts[0])
                    tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                    qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                    R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
                    poses.append(CameraPose(timestamp=t, p_WC=np.array([tx, ty, tz]), R_WC=R))

        return CameraTrajectory(poses, name=os.path.basename(filepath))
