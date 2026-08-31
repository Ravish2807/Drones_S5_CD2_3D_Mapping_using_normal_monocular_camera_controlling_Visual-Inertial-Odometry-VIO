import numpy as np
from typing import Tuple


class FrameTransforms:
    """
    Centralized spatial transformation manager.
    Encapsulates all frame conventions and transformations between:
    - W: VIO World reference frame (+Z up, gravity = [0, 0, -9.81])
    - B: Body / IMU sensing frame (+X forward, +Y left, +Z up)
    - C: Camera optical frame (+Z optical depth forward, +X right, +Y down)
    """

    def __init__(self, R_BC: np.ndarray, p_BC: np.ndarray):
        self.R_BC = np.asarray(R_BC, dtype=np.float64) # Camera -> Body rotation
        self.p_BC = np.asarray(p_BC, dtype=np.float64) # Camera origin in Body coordinates
        self.R_CB = self.R_BC.T
        self.p_CB = -self.R_CB @ self.p_BC

    def body_to_world_point(self, P_B: np.ndarray, p_WB: np.ndarray, R_WB: np.ndarray) -> np.ndarray:
        """Transforms point from Body to World frame: P_W = R_WB * P_B + p_WB"""
        return R_WB @ P_B + p_WB

    def world_to_body_point(self, P_W: np.ndarray, p_WB: np.ndarray, R_WB: np.ndarray) -> np.ndarray:
        """Transforms point from World to Body frame: P_B = R_WB^T * (P_W - p_WB)"""
        return R_WB.T @ (P_W - p_WB)

    def camera_to_body_point(self, P_C: np.ndarray) -> np.ndarray:
        """Transforms point from Camera to Body frame: P_B = R_BC * P_C + p_BC"""
        return self.R_BC @ P_C + self.p_BC

    def body_to_camera_point(self, P_B: np.ndarray) -> np.ndarray:
        """Transforms point from Body to Camera frame: P_C = R_BC^T * (P_B - p_BC)"""
        return self.R_CB @ (P_B - self.p_BC)

    def camera_to_world_pose(self, p_WB: np.ndarray, R_WB: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes Camera pose in World frame (T_WC = T_WB * T_BC).
        Returns: (p_WC, R_WC)
        """
        p_WC = R_WB @ self.p_BC + p_WB
        R_WC = R_WB @ self.R_BC
        return p_WC, R_WC

    def world_to_camera_point(self, P_W: np.ndarray, p_WC: np.ndarray, R_CW: np.ndarray) -> np.ndarray:
        """Transforms point from World to Camera frame: P_C = R_CW * (P_W - p_WC)"""
        return R_CW @ (P_W - p_WC)
