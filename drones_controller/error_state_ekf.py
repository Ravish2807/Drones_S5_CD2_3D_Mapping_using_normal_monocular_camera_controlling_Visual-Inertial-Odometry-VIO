"""
===============================================================================
Model 1: Error-State Extended Kalman Filter (ES-EKF)
===============================================================================
Implements Block 4 from Model 1:
- State vector: x = (p, v, R, ω) ∈ R^3 × R^3 × SO(3) × R^3
- Error state: δx = [δp; δv; δθ; δω] ∈ R^12
- Predicts with nonlinear plant dynamics and propagates covariance P using F = I + A*dt
- Corrects with pose & gyro measurements using SO(3) manifold addition (R ← R * Exp(δθ))
===============================================================================
"""

import numpy as np
from .state_adapter import hat, vee, so3_exp, so3_log, validate_rotation_matrix


class ErrorStateEKF:

    def __init__(
        self,
        mass: float = 1.5,
        gravity: float = 9.81,
        inertia=None,
        q_pos: float = 1e-3,
        q_vel: float = 1e-3,
        q_rot: float = 1e-4,
        q_omega: float = 1e-4,
        r_pos: float = 1e-2,
        r_rot: float = 1e-3,
        r_omega: float = 1e-3
    ):
        self.mass = float(mass)
        self.gravity = float(gravity)
        self.g_vec = np.array([0.0, 0.0, -self.gravity], dtype=float)
        self.e3 = np.array([0.0, 0.0, 1.0], dtype=float)

        if inertia is None:
            inertia = [0.02, 0.02, 0.04]
        self.J = np.diag(np.array(inertia, dtype=float))
        self.J_inv = np.linalg.inv(self.J)

        # Nominal states
        self.p = np.zeros(3, dtype=float)
        self.v = np.zeros(3, dtype=float)
        self.R = np.eye(3, dtype=float)
        self.omega = np.zeros(3, dtype=float)

        # Covariance P ∈ R^(12x12)
        self.P = np.eye(12, dtype=float) * 0.05

        # Process noise covariance Q
        q_diag = np.concatenate([
            np.full(3, q_pos),
            np.full(3, q_vel),
            np.full(3, q_rot),
            np.full(3, q_omega)
        ])
        self.Q = np.diag(q_diag)

        # Measurement noise covariance Rn ∈ R^(9x9) (pos: 3, rot: 3, omega: 3)
        r_diag = np.concatenate([
            np.full(3, r_pos),
            np.full(3, r_rot),
            np.full(3, r_omega)
        ])
        self.Rn = np.diag(r_diag)

        # Measurement matrix H ∈ R^(9x12)
        self.H = np.zeros((9, 12), dtype=float)
        self.H[0:3, 0:3] = np.eye(3)    # δp
        self.H[3:6, 6:9] = np.eye(3)    # δθ
        self.H[6:9, 9:12] = np.eye(3)   # δω

        self.initialized = False

    def initialize(self, p: np.ndarray, v: np.ndarray, R: np.ndarray, omega: np.ndarray):
        self.p = np.array(p, dtype=float).copy()
        self.v = np.array(v, dtype=float).copy()
        self.R = np.array(R, dtype=float).copy()
        self.omega = np.array(omega, dtype=float).copy()
        self.P = np.eye(12, dtype=float) * 0.05
        self.initialized = True

    def predict(self, T: float, tau: np.ndarray, dt: float):
        """
        Nonlinear continuous plant propagation + discrete Jacobian F = I + A*dt.
        u = [T; tau] ∈ R^4
        """
        if dt <= 0.0 or dt > 0.5:
            return

        dt = float(dt)
        T = float(max(0.0, T))
        tau = np.asarray(tau, dtype=float)

        # 1. Propagate nominal state x_hat
        p_next = self.p + self.v * dt
        v_next = self.v + (self.g_vec + (T / self.mass) * (self.R @ self.e3)) * dt
        R_next = self.R @ so3_exp(self.omega * dt)

        # Gyroscopic dynamics
        coriolis = np.cross(self.omega, self.J @ self.omega)
        omega_dot = self.J_inv @ (tau - coriolis)
        omega_next = self.omega + omega_dot * dt

        self.p = p_next
        self.v = v_next
        self.R = R_next
        self.omega = omega_next

        # 2. Continuous Jacobian A = ∂f/∂x ∈ R^(12x12)
        A = np.zeros((12, 12), dtype=float)
        A[0:3, 3:6] = np.eye(3)
        A[3:6, 6:9] = -(T / self.mass) * (self.R @ hat(self.e3))
        A[6:9, 6:9] = -hat(self.omega)
        A[6:9, 9:12] = np.eye(3)
        A[9:12, 9:12] = self.J_inv @ (-hat(self.omega) @ self.J + hat(self.J @ self.omega))

        # 3. Discrete error transition matrix F = I + A*dt
        F = np.eye(12, dtype=float) + A * dt

        # 4. Propagate covariance P = F*P*F^T + Q
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, z_p: np.ndarray, z_R: np.ndarray, z_omega: np.ndarray):
        """
        Measurement correction with sensor observation:
        z = (z_p, z_R, z_omega)
        """
        if not self.initialized:
            self.initialize(z_p, np.zeros(3), z_R, z_omega)
            return

        # 1. Innovation residual on Lie group SO(3)
        r_p = np.asarray(z_p, dtype=float) - self.p
        r_theta = so3_log(self.R.T @ z_R)
        r_omega = np.asarray(z_omega, dtype=float) - self.omega
        residual = np.concatenate([r_p, r_theta, r_omega])

        # 2. Kalman Gain K = P*H^T * (H*P*H^T + Rn)^(-1)
        S = self.H @ self.P @ self.H.T + self.Rn
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 3. Error state correction δx = K * residual
        dx = K @ residual

        # 4. State injection (manifold addition x ← x ⊞ δx)
        self.p += dx[0:3]
        self.v += dx[3:6]
        self.R = self.R @ so3_exp(dx[6:9])
        self.omega += dx[9:12]

        # Orthonormalize rotation matrix
        U, _, Vt = np.linalg.svd(self.R)
        self.R = U @ Vt

        # 5. Joseph form covariance update for numerical symmetry & positive definiteness
        I_KH = np.eye(12, dtype=float) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.Rn @ K.T

    def get_state(self):
        return {
            "p": self.p.copy(),
            "v": self.v.copy(),
            "R": self.R.copy(),
            "omega": self.omega.copy(),
            "P": self.P.copy(),
            "trace_P": float(np.trace(self.P))
        }
