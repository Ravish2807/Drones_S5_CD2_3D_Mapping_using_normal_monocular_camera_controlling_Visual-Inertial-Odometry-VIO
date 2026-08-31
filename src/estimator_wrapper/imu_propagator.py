import numpy as np
from typing import Dict, Any, Tuple
from .math_utils import skew_symmetric, exp_so3, rot_to_quat, quat_to_rot


class IMUPropagator:
    """
    Implements 6-DoF inertial state propagation and error-state covariance
    kinematics for visual-inertial odometry.
    """

    def __init__(self, config: Dict[str, Any]):
        self.cfg = config
        imu_cfg = config.get("imu", {})
        est_cfg = config.get("estimator", {})

        # Continuous noise densities
        self.sigma_a = imu_cfg.get("noise_accel", 0.005)
        self.sigma_g = imu_cfg.get("noise_gyro", 0.001)
        self.sigma_ba = imu_cfg.get("random_walk_accel", 0.0001)
        self.sigma_bg = imu_cfg.get("random_walk_gyro", 0.00001)

        # Gravity in world frame
        self.gravity = np.array(est_cfg.get("gravity", [0.0, 0.0, -9.81]), dtype=np.float64)

        # Continuous process noise covariance matrix Qc (12x12)
        # Noise order: [n_a (3), n_g (3), n_ba (3), n_bg (3)]
        self.Q_c = np.diag([
            self.sigma_a**2, self.sigma_a**2, self.sigma_a**2,
            self.sigma_g**2, self.sigma_g**2, self.sigma_g**2,
            self.sigma_ba**2, self.sigma_ba**2, self.sigma_ba**2,
            self.sigma_bg**2, self.sigma_bg**2, self.sigma_bg**2
        ])

    def propagate_state(self, p: np.ndarray, v: np.ndarray, R: np.ndarray,
                        ba: np.ndarray, bg: np.ndarray,
                        accel: np.ndarray, gyro: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Nominal state propagation using Runge-Kutta 4th Order / Midpoint Integration.
        """
        # Unbiased inertial measurements
        acc_unbiased = accel - ba
        gyro_unbiased = gyro - bg

        # 1. Orientation update: R_{k+1} = R_k * Exp((omega - bg) * dt)
        delta_rot = exp_so3(gyro_unbiased * dt)
        R_next = R @ delta_rot

        # 2. Acceleration in world frame: a_W = R * (a_m - ba) + g
        # Midpoint rotation for velocity and position integration
        R_mid = R @ exp_so3(gyro_unbiased * 0.5 * dt)
        a_W_mid = R_mid @ acc_unbiased + self.gravity

        # 3. Position and velocity update
        p_next = p + v * dt + 0.5 * a_W_mid * (dt ** 2)
        v_next = v + a_W_mid * dt

        return p_next, v_next, R_next

    def get_state_transition_and_noise(self, R: np.ndarray, ba: np.ndarray, bg: np.ndarray,
                                       accel: np.ndarray, gyro: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes the 15x15 discrete state transition matrix Phi and discrete noise covariance Qd.
        Error state order: [delta_p (3), delta_v (3), delta_theta (3), delta_ba (3), delta_bg (3)]
        """
        acc_unbiased = accel - ba
        gyro_unbiased = gyro - bg

        F = np.zeros((15, 15), dtype=np.float64)
        
        # d(delta_p)/dt = delta_v
        F[0:3, 3:6] = np.eye(3)

        # d(delta_v)/dt = -R * [a_unbiased x] * delta_theta - R * delta_ba
        F[3:6, 6:9] = -R @ skew_symmetric(acc_unbiased)
        F[3:6, 9:12] = -R

        # d(delta_theta)/dt = -[gyro_unbiased x] * delta_theta - delta_bg
        F[6:9, 6:9] = -skew_symmetric(gyro_unbiased)
        F[6:9, 12:15] = -np.eye(3)

        # Discrete state transition via Taylor expansion: Phi = I + F*dt + 0.5*F^2*dt^2
        F_dt = F * dt
        Phi = np.eye(15, dtype=np.float64) + F_dt + 0.5 * (F_dt @ F_dt)

        # Continuous noise input matrix G (15x12)
        G = np.zeros((15, 12), dtype=np.float64)
        G[3:6, 0:3] = -R           # accel noise into velocity
        G[6:9, 3:6] = -np.eye(3)   # gyro noise into orientation
        G[9:12, 6:9] = np.eye(3)   # accel random walk
        G[12:15, 9:12] = np.eye(3) # gyro random walk

        # Discrete process noise covariance: Qd = Phi * G * Qc * G^T * Phi^T * dt
        Q_d = Phi @ G @ self.Q_c @ G.T @ Phi.T * dt

        return Phi, Q_d
