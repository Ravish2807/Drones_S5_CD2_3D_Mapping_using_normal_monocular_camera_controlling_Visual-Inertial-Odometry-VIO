"""
===============================================================================
Model 2: Optimal Control Engine (DARE, LQR, and Constrained Stacked MPC)
===============================================================================
Implements Blocks 3–9 from Model 2:
- LQR Branch (infinite horizon):
    DARE: P = A^T*P*A - A^T*P*B*(R + B^T*P*B)^(-1)*B^T*P*A + Q
    Gain: K = (R + B^T*P*B)^(-1)*B^T*P*A ∈ R^(4x12)
    Input: u_k = -K * x_k
- MPC Branch (finite horizon N=25 + constraints):
    min_U  1/2 * U^T * H * U + f^T * U
    s.t.   u_min <= u_k <= u_max
    H = 2 * (Su^T * Q_bar * Su + R_bar)
    f = 2 * Su^T * Q_bar * Sx * x0
    Apply receding horizon: apply u0*, re-solve at k+1
===============================================================================
"""

import numpy as np
import warnings
with warnings.catch_warnings():
    warnings.simplefilter('ignore', UserWarning)
    try:
        import scipy.linalg
        HAS_SCIPY = True
    except ImportError:
        HAS_SCIPY = False


def _block_diag(*matrices):
    shapes = [m.shape for m in matrices]
    out = np.zeros((sum(s[0] for s in shapes), sum(s[1] for s in shapes)), dtype=float)
    r, c = 0, 0
    for m, (mr, mc) in zip(matrices, shapes):
        out[r:r+mr, c:c+mc] = m
        r += mr
        c += mc
    return out


class OptimalControlEngine:

    def __init__(
        self,
        A: np.ndarray,
        B: np.ndarray,
        q_weights=None,
        r_weights=None,
        horizon: int = 25,
        dt: float = 0.04,
        max_vel: float = 1.0,
        max_tilt_rad: float = 0.35,  # 20 degrees
        t_min: float = 5.0,
        t_max: float = 25.0,
        mg0: float = 14.715
    ):
        self.A = np.array(A, dtype=float).copy()
        self.B = np.array(B, dtype=float).copy()
        self.horizon = int(horizon)
        self.dt = float(dt)
        self.mg0 = float(mg0)

        # Dimension checks
        self.nx = 12
        self.nu = 4

        # Cost weights Q ∈ R^(12x12), R ∈ R^(4x4)
        if q_weights is None:
            # Default from Model 2 specification
            q_diag = np.array([
                10.0, 10.0, 20.0,   # Position error (x, y, z)
                2.0,  2.0,  5.0,    # Velocity error (vx, vy, vz)
                5.0,  5.0,  2.0,    # Rotation vector Log(R) (roll, pitch, yaw)
                0.5,  0.5,  0.2     # Angular rate (p, q, r)
            ], dtype=float)
        else:
            q_diag = np.array(q_weights, dtype=float)
        self.Q = np.diag(q_diag)

        if r_weights is None:
            # Input deviation weights [ΔT, τx, τy, τz]
            r_diag = np.array([0.1, 1.0, 1.0, 1.0], dtype=float)
        else:
            r_diag = np.array(r_weights, dtype=float)
        self.R = np.diag(r_diag)

        # Limits on control deviation u = [ΔT; τ]
        # 5 N <= T <= 25 N  ==>  5 - 14.715 <= ΔT <= 25 - 14.715
        self.u_min = np.array([t_min - self.mg0, -0.5, -0.5, -0.5], dtype=float)
        self.u_max = np.array([t_max - self.mg0,  0.5,  0.5,  0.5], dtype=float)

        self.P = None
        self.K = None
        self.H = None
        self.H_inv = None
        self.Sx = None
        self.Su = None
        self.Q_bar = None
        self.R_bar = None

        # Solve steady-state DARE and compile MPC prediction matrices
        self.update_model(self.A, self.B)

    def update_model(self, A: np.ndarray, B: np.ndarray):
        """
        Updates learned/identified (A, B) matrices and re-solves DARE & MPC Hessian.
        """
        self.A = np.array(A, dtype=float).copy()
        self.B = np.array(B, dtype=float).copy()

        # 1. Steady-state DARE: P = A^T*P*A - A^T*P*B*(R + B^T*P*B)^(-1)*B^T*P*A + Q
        self.P = None
        if HAS_SCIPY:
            try:
                self.P = scipy.linalg.solve_discrete_are(self.A, self.B, self.Q, self.R)
            except Exception:
                self.P = None
        if self.P is None:
            # Fallback to Riccati recursion (Block 5)
            P = self.Q.copy()
            for _ in range(250):
                P_next = self.A.T @ P @ self.A - self.A.T @ P @ self.B @ np.linalg.inv(self.R + self.B.T @ P @ self.B) @ self.B.T @ P @ self.A + self.Q
                if np.max(np.abs(P_next - P)) < 1e-6:
                    P = P_next
                    break
                P = P_next
            self.P = P

        # 2. LQR Gain K ∈ R^(4x12) (Block 4)
        R_BPB = self.R + self.B.T @ self.P @ self.B
        self.K = np.linalg.inv(R_BPB) @ self.B.T @ self.P @ self.A

        # 3. Stacked MPC Prediction Matrices Sx ∈ R^(12N x 12), Su ∈ R^(12N x 4N) (Block 8)
        N = self.horizon
        nx = self.nx
        nu = self.nu

        self.Sx = np.zeros((nx * N, nx), dtype=float)
        self.Su = np.zeros((nx * N, nu * N), dtype=float)

        Ak = np.eye(nx, dtype=float)
        for i in range(N):
            Ak = Ak @ self.A
            self.Sx[i*nx:(i+1)*nx, :] = Ak

        for i in range(N):
            for j in range(i + 1):
                A_power = np.linalg.matrix_power(self.A, i - j)
                self.Su[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = A_power @ self.B

        # Terminal weight Pf = P from DARE
        if HAS_SCIPY:
            self.Q_bar = scipy.linalg.block_diag(*([self.Q] * (N - 1) + [self.P]))
            self.R_bar = scipy.linalg.block_diag(*([self.R] * N))
        else:
            self.Q_bar = _block_diag(*([self.Q] * (N - 1) + [self.P]))
            self.R_bar = _block_diag(*([self.R] * N))

        # Stacked QP Hessian H = 2 * (Su^T * Q_bar * Su + R_bar)
        self.H = 2.0 * (self.Su.T @ self.Q_bar @ self.Su + self.R_bar)
        self.H_inv = np.linalg.inv(self.H)

    def compute_lqr(self, x0: np.ndarray) -> np.ndarray:
        """
        LQR control law (Blocks 4 & 10):
        u_k = -K * x_k
        """
        x0 = np.asarray(x0, dtype=float)
        u_lqr = -self.K @ x0
        # Clip to admissible input limits
        return np.clip(u_lqr, self.u_min, self.u_max)

    def compute_mpc(self, x0: np.ndarray, max_iter: int = 15) -> np.ndarray:
        """
        Constrained Stacked QP MPC (Blocks 7, 8, 9, 10):
        min_U 1/2 U^T H U + f^T U  s.t.  U_min <= U <= U_max
        Uses fast projected gradient descent starting from unconstrained optimum.
        """
        x0 = np.asarray(x0, dtype=float)

        # Linear term f = 2 * Su^T * Q_bar * Sx * x0
        f = 2.0 * (self.Su.T @ self.Q_bar @ (self.Sx @ x0))

        # Unconstrained optimum U_unc = -H^(-1) * f
        U = -self.H_inv @ f

        # Stacked box bounds for horizon N
        U_min = np.tile(self.u_min, self.horizon)
        U_max = np.tile(self.u_max, self.horizon)

        # Fast projected gradient refinement for constraints
        if np.any(U < U_min) or np.any(U > U_max):
            U = np.clip(U, U_min, U_max)
            # 5-10 projected gradient steps with Barzilai-Borwein step size
            step_size = 1.0 / (np.linalg.norm(self.H, ord=2) + 1e-4)
            for _ in range(max_iter):
                grad = self.H @ U + f
                if np.max(np.abs(grad)) < 1e-3:
                    break
                U = np.clip(U - step_size * grad, U_min, U_max)

        # Receding horizon principle: Extract first control input u0* ∈ R^4
        u0_star = U[0:4]
        return u0_star

    def compute_value_function(self, x: np.ndarray) -> float:
        """
        Evaluates quadratic value function V(x) = x^T * P * x
        """
        x = np.asarray(x, dtype=float)
        return float(x.T @ self.P @ x)
