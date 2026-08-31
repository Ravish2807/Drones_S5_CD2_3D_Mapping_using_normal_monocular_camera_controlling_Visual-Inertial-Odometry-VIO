import os
import yaml
import numpy as np
from typing import Dict, Any, Optional, Callable
from .msckf_estimator import MSCKFEstimator
from .math_utils import rot_to_quat


class VIOStreamInterface:
    """
    Standardized, modular VIO streaming interface for deployment to ROS 2 nodes,
    live hardware sensor pipelines (RealSense, drone companion computers), and Gazebo topics.
    """

    def __init__(self, config_path: str):
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"VIO config file not found: {config_path}")

        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)

        self.estimator = MSCKFEstimator(self.config)
        self.odometry_callback: Optional[Callable[[Dict[str, Any]], None]] = None

    def register_odometry_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """Registers a callback function invoked whenever a new state estimate is computed."""
        self.odometry_callback = callback

    def push_imu(self, timestamp_s: float, accel_m_s2: np.ndarray, gyro_rad_s: np.ndarray):
        """
        Receives 6-axis IMU measurement (typically 100 - 400 Hz).
        - accel_m_s2: [ax, ay, az] in m/s^2
        - gyro_rad_s: [wx, wy, wz] in rad/s
        """
        self.estimator.process_imu(
            timestamp=float(timestamp_s),
            linear_accel=np.asarray(accel_m_s2, dtype=np.float64),
            angular_vel=np.asarray(gyro_rad_s, dtype=np.float64)
        )

    def push_camera_frame(self, timestamp_s: float, image: np.ndarray) -> Optional[Dict[str, Any]]:
        """
        Receives synchronized camera frame (typically 20 - 30 Hz).
        Returns the updated 6-DoF drone state dictionary.
        """
        state_res = self.estimator.process_camera(float(timestamp_s), image)

        if state_res.get("status") == "uninitialized":
            return None

        # Build standardized ROS 2 / Odometry message payload
        odom_msg = {
            "timestamp": state_res["timestamp"],
            "position": state_res["position"].tolist(),        # [x, y, z] in world frame (m)
            "orientation_q": state_res["orientation_q"].tolist(), # [w, x, y, z] Hamilton quaternion
            "velocity": state_res["velocity"].tolist(),        # [vx, vy, vz] in world frame (m/s)
            "bias_accel": state_res["bias_accel"].tolist(),    # [bax, bay, baz]
            "bias_gyro": state_res["bias_gyro"].tolist(),      # [bgx, bgy, bgz]
            "active_features": state_res["active_features"],
            "updated_features": state_res["updated_features"],
            "latency_ms": state_res["proc_time_sec"] * 1000.0
        }

        if self.odometry_callback:
            self.odometry_callback(odom_msg)

        return odom_msg
