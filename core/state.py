import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Dict, Any


@dataclass
class VIOState:
    """
    Standardized, self-contained 6-DoF VIO Motion State data contract.
    Decoupled from all dataset loaders, ROS dependencies, or simulation internals.
    """
    timestamp: float
    position: np.ndarray             # [x, y, z] in World frame (m)
    velocity: np.ndarray             # [vx, vy, vz] in World frame (m/s)
    orientation_R: np.ndarray        # 3x3 SO(3) rotation matrix (World <- Body)
    orientation_q: np.ndarray        # [w, x, y, z] unit quaternion (Hamilton convention)
    bias_accel: np.ndarray           # [bax, bay, baz] in Body frame (m/s^2)
    bias_gyro: np.ndarray            # [bgx, bgy, bgz] in Body frame (rad/s)
    active_features: int = 0
    updated_features: int = 0
    clones_count: int = 0
    covariance_diag: Optional[np.ndarray] = None
    quality_score: float = 1.0       # 0.0 (lost/diverged) to 1.0 (healthy tracking)
    latency_sec: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": float(self.timestamp),
            "position": self.position.tolist(),
            "velocity": self.velocity.tolist(),
            "orientation_q": self.orientation_q.tolist(),
            "orientation_euler_deg": self.get_euler_degrees().tolist(),
            "bias_accel": self.bias_accel.tolist(),
            "bias_gyro": self.bias_gyro.tolist(),
            "active_features": int(self.active_features),
            "updated_features": int(self.updated_features),
            "clones_count": int(self.clones_count),
            "latency_ms": float(self.latency_sec * 1000.0),
            "quality_score": float(self.quality_score)
        }

    def get_euler_degrees(self) -> np.ndarray:
        """Returns [roll, pitch, yaw] in degrees (XYZ convention)."""
        from .math_utils import rot_to_euler_deg
        return rot_to_euler_deg(self.orientation_R)
