import os
import cv2
import yaml
import numpy as np
import pandas as pd
from typing import Dict, Any, Generator, Optional, Tuple


class EuRoCLoader:
    """
    Standard EuRoC MAV ASL dataset stream loader.
    Reads synchronized cam0/cam1 images, 200 Hz IMU data, and ground truth state.
    """

    def __init__(self, sequence_path: str, config: Optional[Dict[str, Any]] = None):
        self.sequence_path = os.path.abspath(sequence_path)
        self.config = config or {}

        self.mav0_dir = os.path.join(self.sequence_path, "mav0") if os.path.exists(os.path.join(self.sequence_path, "mav0")) else self.sequence_path
        self.imu0_dir = os.path.join(self.mav0_dir, "imu0")
        self.cam0_dir = os.path.join(self.mav0_dir, "cam0")
        self.gt_dir = os.path.join(self.mav0_dir, "state_groundtruth_estimate0")

        self.imu_data: Optional[pd.DataFrame] = None
        self.cam0_data: Optional[pd.DataFrame] = None
        self.gt_data: Optional[pd.DataFrame] = None

        self._load_metadata()

    def _load_metadata(self):
        imu_csv = os.path.join(self.imu0_dir, "data.csv")
        if os.path.exists(imu_csv):
            self.imu_data = pd.read_csv(imu_csv, comment='#', header=None,
                                        names=['timestamp', 'w_x', 'w_y', 'w_z', 'a_x', 'a_y', 'a_z'])
            self.imu_data['timestamp_sec'] = self.imu_data['timestamp'] * 1e-9

        cam0_csv = os.path.join(self.cam0_dir, "data.csv")
        if os.path.exists(cam0_csv):
            self.cam0_data = pd.read_csv(cam0_csv, comment='#', header=None, names=['timestamp', 'filename'])
            self.cam0_data['timestamp_sec'] = self.cam0_data['timestamp'] * 1e-9

        gt_csv = os.path.join(self.gt_dir, "data.csv")
        if os.path.exists(gt_csv):
            self.gt_data = pd.read_csv(gt_csv, comment='#', header=None,
                                       names=['timestamp', 'p_x', 'p_y', 'p_z', 'q_w', 'q_x', 'q_y', 'q_z',
                                              'v_x', 'v_y', 'v_z', 'b_w_x', 'b_w_y', 'b_w_z', 'b_a_x', 'b_a_y', 'b_a_z'])
            self.gt_data['timestamp_sec'] = self.gt_data['timestamp'] * 1e-9

    def stream_sensor_data(self) -> Generator[Dict[str, Any], None, None]:
        """Chronologically yields IMU and Camera packets."""
        if self.imu_data is None or self.cam0_data is None:
            return

        imu_idx = 0
        cam_idx = 0
        n_imu = len(self.imu_data)
        n_cam = len(self.cam0_data)

        while imu_idx < n_imu or cam_idx < n_cam:
            t_imu = self.imu_data.iloc[imu_idx]['timestamp_sec'] if imu_idx < n_imu else float('inf')
            t_cam = self.cam0_data.iloc[cam_idx]['timestamp_sec'] if cam_idx < n_cam else float('inf')

            if t_imu <= t_cam:
                row = self.imu_data.iloc[imu_idx]
                yield {
                    "type": "imu",
                    "timestamp": row['timestamp_sec'],
                    "linear_accel": np.array([row['a_x'], row['a_y'], row['a_z']]),
                    "angular_vel": np.array([row['w_x'], row['w_y'], row['w_z']])
                }
                imu_idx += 1
            else:
                row = self.cam0_data.iloc[cam_idx]
                img_path = os.path.join(self.cam0_dir, "data", row['filename'])
                img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE) if os.path.exists(img_path) else np.zeros((480, 752), dtype=np.uint8)
                yield {
                    "type": "camera",
                    "timestamp": row['timestamp_sec'],
                    "image": img
                }
                cam_idx += 1
