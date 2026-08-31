import os
import yaml
import numpy as np
import pandas as pd
from typing import Dict, Any, Optional, Callable, List

from core.msckf_estimator import MSCKFEstimator
from core.state import VIOState
from .packets import IMUPacket, ImagePacket


class VIOInterface:
    """
    Standardized, OS-agnostic Adapter Boundary for the VIO Core.
    Acts as the single point of entry whether data originates from:
    - Synthetic simulation (Windows/Linux)
    - Public benchmark playback (EuRoC, TUM-VI)
    - Live ROS 2 Nodes (Ubuntu 22.04 + Humble)
    - Gazebo topic bridges or Companion Computer hardware streams
    """

    def __init__(self, config_or_path: Any):
        if isinstance(config_or_path, str):
            if not os.path.exists(config_or_path):
                raise FileNotFoundError(f"Config file not found: {config_or_path}")
            with open(config_or_path, "r") as f:
                self.config = yaml.safe_load(f)
        elif isinstance(config_or_path, dict):
            self.config = config_or_path
        else:
            raise ValueError("config_or_path must be a file path or a dictionary.")

        self.estimator = MSCKFEstimator(self.config)
        self.state_history: List[VIOState] = []
        self.state_callbacks: List[Callable[[VIOState], None]] = []

        # Runtime telemetry
        self.total_imu_samples = 0
        self.total_cam_frames = 0
        self.total_imu_proc_time = 0.0
        self.total_cam_proc_time = 0.0

    def register_callback(self, callback: Callable[[VIOState], None]):
        """Registers a listener callback invoked upon each valid state estimate."""
        self.state_callbacks.append(callback)

    def push_imu(self, timestamp: float,
                 ax: float, ay: float, az: float,
                 gx: float, gy: float, gz: float):
        """Standard float-based IMU feed."""
        accel = np.array([ax, ay, az], dtype=np.float64)
        gyro = np.array([gx, gy, gz], dtype=np.float64)
        self.estimator.process_imu(float(timestamp), accel, gyro)
        self.total_imu_samples += 1

    def push_imu_packet(self, packet: IMUPacket):
        """Packet-based IMU feed."""
        self.estimator.process_imu(packet.timestamp, packet.accel, packet.gyro)
        self.total_imu_samples += 1

    def push_camera_frame(self, timestamp: float, image: np.ndarray) -> Optional[VIOState]:
        """Standard image-based camera frame feed."""
        state = self.estimator.process_camera(float(timestamp), image)
        self.total_cam_frames += 1

        if state is not None:
            self.state_history.append(state)
            for cb in self.state_callbacks:
                cb(state)

        return state

    def push_camera_packet(self, packet: ImagePacket) -> Optional[VIOState]:
        """Packet-based camera frame feed."""
        return self.push_camera_frame(packet.timestamp, packet.image)

    def get_current_state(self) -> Optional[VIOState]:
        """Returns the most recent 6-DoF VIO motion state."""
        return self.estimator.current_state

    def export_state_csv(self, filepath: str):
        """
        Exports standardized trajectory output to CSV format.
        Format: timestamp,px,py,pz,vx,vy,vz,qw,qx,qy,qz,bax,bay,baz,bgx,bgy,bgz
        """
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        records = []
        for s in self.state_history:
            records.append([
                s.timestamp,
                s.position[0], s.position[1], s.position[2],
                s.velocity[0], s.velocity[1], s.velocity[2],
                s.orientation_q[0], s.orientation_q[1], s.orientation_q[2], s.orientation_q[3],
                s.bias_accel[0], s.bias_accel[1], s.bias_accel[2],
                s.bias_gyro[0], s.bias_gyro[1], s.bias_gyro[2]
            ])

        header = "timestamp,px,py,pz,vx,vy,vz,qw,qx,qy,qz,bax,bay,baz,bgx,bgy,bgz"
        np.savetxt(filepath, np.array(records), delimiter=",", header=header, comments="")
        print(f"Exported {len(records)} states to {filepath}")
