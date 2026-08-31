import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class IMUPacket:
    """
    Standardized IMU measurement packet.
    Compatible with ROS 2 sensor_msgs/msg/Imu or hardware IMU drivers.
    """
    timestamp: float
    accel: np.ndarray      # [ax, ay, az] in m/s^2 (Body frame)
    gyro: np.ndarray       # [gx, gy, gz] in rad/s (Body frame)
    frame_id: str = "imu_link"

    def __post_init__(self):
        self.accel = np.asarray(self.accel, dtype=np.float64)
        self.gyro = np.asarray(self.gyro, dtype=np.float64)


@dataclass
class ImagePacket:
    """
    Standardized Camera frame packet.
    Compatible with ROS 2 sensor_msgs/msg/Image (via cv_bridge) or OpenCV camera streams.
    """
    timestamp: float
    image: np.ndarray      # Grayscale or BGR image
    frame_id: str = "camera_optical_link"
    seq: int = 0
