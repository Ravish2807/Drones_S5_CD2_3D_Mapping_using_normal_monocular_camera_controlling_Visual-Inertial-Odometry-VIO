import numpy as np
from typing import Dict, Any, Tuple
from .math_utils import exp_so3, skew_symmetric


class IMUPropagator:
    """
    Continuous-discrete 6-DoF inertial state & error covariance propagator.
    Operates strictly on explicit measurement timestamps and dynamic dt.
    """

    def __init__(self, config: Dict[str, Any]):
        imu_cfg = config.get("imu", {})
        
        self.noise_accel = float(imu_cfg.get("noise_accel", 0.005))
        self.noise_gyro = float(imu_cfg.get("noise_gyro", 0.001))
        self.rw_accel = float(imu_cfg.get("random_walk_accel", 0.0001))
        self.rw_gyro = float(imu_cfg.get("random_walk_gyro", 0.00001))
        
        # Gravity vector in VIO World frame
        self.gravity_W = np.array(config.get("world", {}).get("gravity", [0.0, 0.0, -9.81]), dtype=np.float64)

        # Continuous noise spectral density matrix Qc (12x12)
        self.Qc = np.zeros((12, 12), dtype=np.float64)
        self.Qc[0:3, 0:3] = np.eye(3) * (self.noise_accel ** 2)
        self.Qc[3:6, 3:6] = np.eye(3) * (self.noise_gyro ** 2)
        self.Qc[6:9, 6:9] = np.eye(3) * (self.rw_accel ** 2)
        self.Qc[9:12, 9:12] = np.eye(3) * (self.rw_gyro ** 2)

    def propagate_state(self,
                        p: np.ndarray,
                        v: np.ndarray,
                        R: np.ndarray,
                        ba: np.ndarray,
                        bg: np.ndarray,
                        acc_meas: np.ndarray,
                        gyro_meas: np.ndarray,
                        dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Propagates 6-DoF state over dynamic timestep dt using midpoint integration.
        """
        if dt <= 0:
            return p, v, R

        # Unbiased inertial rates in Body frame
        w_unbiased = gyro_meas - bg
        a_unbiased = acc_meas - ba

        # Midpoint orientation increment
        delta_R = exp_so3(w_unbiased * dt)
        R_mid = R @ exp_so3(w_unbiased * (0.5 * dt))

        # Specific force in World frame: a_W = R_mid * a_B + g_W
        acc_world = R_mid @ a_unbiased + self.gravity_W

        # Update position and velocity
        p_next = p + v * dt + 0.5 * acc_world * (dt ** 2)
        v_next = v + acc_world * dt
        R_next = R @ delta_R

        return p_next, v_next, R_next

    def get_state_transition_and_noise(self,
                                       R: np.ndarray,
                                       ba: np.ndarray,
                                       bg: np.ndarray,
                                       acc_meas: np.ndarray,
                                       gyro_meas: np.ndarray,
                                       dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes 15x15 discrete state transition matrix Phi and discrete noise Qd.
        State vector: [delta_p (0:3), delta_v (3:6), delta_theta (6:9), delta_ba (9:12), delta_bg (12:15)]
        """
        if dt <= 0:
            return np.eye(15, dtype=np.float64), np.zeros((15, 15), dtype=np.float64)

        w_unbiased = gyro_meas - bg
        a_unbiased = acc_meas - ba

        # Continuous error-state Jacobian F (15x15)
        F = np.zeros((15, 15), dtype=np.float64)
        F[0:3, 3:6] = np.eye(3)
        F[3:6, 6:9] = -R @ skew_symmetric(a_unbiased)
        F[3:6, 9:12] = -R
        F[6:9, 6:9] = -skew_symmetric(w_unbiased)
        F[6:9, 12:15] = -np.eye(3)

        # Continuous noise input matrix G (15x12)
        G = np.zeros((15, 12), dtype=np.float64)
        G[3:6, 0:3] = -R
        G[6:9, 3:6] = -np.eye(3)
        G[9:12, 6:9] = np.eye(3)
        G[12:15, 9:12] = np.eye(3)

        # First-order discretization
        Phi = np.eye(15, dtype=np.float64) + F * dt + 0.5 * (F @ F) * (dt ** 2)
        Qd = (G @ self.Qc @ G.T) * dt

        return Phi, Qd
