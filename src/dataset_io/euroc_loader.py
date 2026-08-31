import os
import glob
import numpy as np
import pandas as pd
import cv2
from typing import Dict, List, Tuple, Generator, Optional, Any


class EuRoCDatasetLoader:
    """
    Data loader for the EuRoC MAV benchmark dataset.
    Parses timestamped stereo images (cam0/cam1), 200 Hz IMU (imu0), and ground truth trajectories.
    """

    def __init__(self, sequence_path: str):
        self.sequence_path = sequence_path
        self.mav0_path = os.path.join(sequence_path, "mav0") if os.path.isdir(os.path.join(sequence_path, "mav0")) else sequence_path
        
        self.cam0_dir = os.path.join(self.mav0_path, "cam0")
        self.cam1_dir = os.path.join(self.mav0_path, "cam1")
        self.imu_dir = os.path.join(self.mav0_path, "imu0")
        self.gt_dir = os.path.join(self.mav0_path, "state_groundtruth_estimate0")

        self.imu_data = None
        self.cam0_data = None
        self.cam1_data = None
        self.gt_data = None

        self._load_metadata()

    def _load_metadata(self):
        # Load IMU
        imu_csv = os.path.join(self.imu_dir, "data.csv")
        if os.path.exists(imu_csv):
            # EuRoC IMU format: #timestamp [ns], w_RS_x, w_RS_y, w_RS_z, a_RS_x, a_RS_y, a_RS_z
            self.imu_data = pd.read_csv(imu_csv, comment="#", header=None,
                                        names=["timestamp_ns", "w_x", "w_y", "w_z", "a_x", "a_y", "a_z"])
            self.imu_data["timestamp_s"] = self.imu_data["timestamp_ns"] * 1e-9

        # Load cam0
        cam0_csv = os.path.join(self.cam0_dir, "data.csv")
        if os.path.exists(cam0_csv):
            self.cam0_data = pd.read_csv(cam0_csv, comment="#", header=None,
                                         names=["timestamp_ns", "filename"])
            self.cam0_data["timestamp_s"] = self.cam0_data["timestamp_ns"] * 1e-9

        # Load cam1 (if available)
        cam1_csv = os.path.join(self.cam1_dir, "data.csv")
        if os.path.exists(cam1_csv):
            self.cam1_data = pd.read_csv(cam1_csv, comment="#", header=None,
                                         names=["timestamp_ns", "filename"])
            self.cam1_data["timestamp_s"] = self.cam1_data["timestamp_ns"] * 1e-9

        # Load Ground Truth (if available)
        gt_csv = os.path.join(self.gt_dir, "data.csv")
        if os.path.exists(gt_csv):
            # #timestamp [ns], p_x, p_y, p_z, q_w, q_x, q_y, q_z, v_x, v_y, v_z, b_w_x, b_w_y, b_w_z, b_a_x, b_a_y, b_a_z
            self.gt_data = pd.read_csv(gt_csv, comment="#", header=None,
                                       names=["timestamp_ns", "p_x", "p_y", "p_z",
                                              "q_w", "q_x", "q_y", "q_z",
                                              "v_x", "v_y", "v_z",
                                              "b_w_x", "b_w_y", "b_w_z",
                                              "b_a_x", "b_a_y", "b_a_z"])
            self.gt_data["timestamp_s"] = self.gt_data["timestamp_ns"] * 1e-9

    def get_ground_truth(self) -> Optional[pd.DataFrame]:
        return self.gt_data

    def get_image(self, cam_idx: int, filename: str) -> Optional[np.ndarray]:
        cam_dir = self.cam0_dir if cam_idx == 0 else self.cam1_dir
        img_path = os.path.join(cam_dir, "data", filename)
        if not os.path.exists(img_path):
            return None
        return cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

    def stream_sensor_events(self) -> Generator[Dict[str, Any], None, None]:
        """
        Yields time-ordered sensor events:
        - {"type": "imu", "timestamp": t, "angular_vel": [wx, wy, wz], "linear_accel": [ax, ay, az]}
        - {"type": "camera", "timestamp": t, "cam_id": 0, "image": np.ndarray}
        """
        if self.imu_data is None or self.cam0_data is None:
            raise RuntimeError(f"Missing dataset files in sequence path {self.sequence_path}")

        imu_idx = 0
        cam_idx = 0
        n_imu = len(self.imu_data)
        n_cam = len(self.cam0_data)

        while imu_idx < n_imu or cam_idx < n_cam:
            imu_t = self.imu_data["timestamp_s"].iloc[imu_idx] if imu_idx < n_imu else float("inf")
            cam_t = self.cam0_data["timestamp_s"].iloc[cam_idx] if cam_idx < n_cam else float("inf")

            if imu_t <= cam_t and imu_idx < n_imu:
                row = self.imu_data.iloc[imu_idx]
                yield {
                    "type": "imu",
                    "timestamp": row["timestamp_s"],
                    "angular_vel": np.array([row["w_x"], row["w_y"], row["w_z"]], dtype=np.float64),
                    "linear_accel": np.array([row["a_x"], row["a_y"], row["a_z"]], dtype=np.float64),
                }
                imu_idx += 1
            elif cam_idx < n_cam:
                row = self.cam0_data.iloc[cam_idx]
                img = self.get_image(0, row["filename"])
                yield {
                    "type": "camera",
                    "timestamp": row["timestamp_s"],
                    "cam_id": 0,
                    "image": img,
                    "filename": row["filename"]
                }
                cam_idx += 1
