"""
===============================================================================
Model 2: Dynamic Mode Decomposition with Control (DMDc)
===============================================================================
Implements Blocks 1 & 2 from Model 2:
- State deviation: x_k = [p_k - pref; v_k; Log(R_k); ω_k] ∈ R^12
- Input deviation: u_k = [T_k - mg0; τ_k] ∈ R^4
- Discretized system identification:
    X  = [x_1, x_2, ..., x_{N-1}] ∈ R^(12 x (N-1))
    X+ = [x_2, x_3, ..., x_N]     ∈ R^(12 x (N-1))
    U  = [u_1, u_2, ..., u_{N-1}] ∈ R^(4 x (N-1))
    [A  B] = X+ * [X; U]†  ==>  x_{k+1} ≈ A * x_k + B * u_k
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
from collections import deque
from .state_adapter import so3_log, hat


class DMDcIdentifier:

    def __init__(
        self,
        mass: float = 1.5,
        gravity: float = 9.81,
        inertia=None,
        dt: float = 0.04,
        max_samples: int = 150,
        min_fit_samples: int = 25
    ):
        self.mass = float(mass)
        self.gravity = float(gravity)
        self.mg0 = self.mass * self.gravity
        self.dt = float(dt)

        if inertia is None:
            inertia = [0.02, 0.02, 0.04]
        self.J = np.diag(np.array(inertia, dtype=float))
        self.J_inv = np.linalg.inv(self.J)

        self.max_samples = int(max_samples)
        self.min_fit_samples = int(min_fit_samples)

        # Baseline physical analytical matrices at hover
        self.A_phys, self.B_phys = self._build_physical_linear_model(self.dt)
        self.A = self.A_phys.copy()
        self.B = self.B_phys.copy()

        # Rolling snapshot buffers
        self.x_history = deque(maxlen=self.max_samples)
        self.u_history = deque(maxlen=self.max_samples)

        self.samples_collected = 0
        self.is_identified = False
        self.identification_residual = 0.0

    def _build_physical_linear_model(self, dt: float):
        """
        Continuous first-principles hover linearization:
        δp_dot = δv
        δv_dot = -g * [e3]× * δθ + (1/m)*e3 * δT
        δθ_dot = δω
        δω_dot = J^(-1) * δτ
        """
        Ac = np.zeros((12, 12), dtype=float)
        Ac[0:3, 3:6] = np.eye(3)
        Ac[3, 7] = self.gravity     # +g * θ_y
        Ac[4, 6] = -self.gravity    # -g * θ_x
        Ac[6:9, 9:12] = np.eye(3)

        Bc = np.zeros((12, 4), dtype=float)
        Bc[5, 0] = 1.0 / self.mass  # δT
        Bc[9:12, 1:4] = self.J_inv  # δτ

        # Exact matrix exponential discretization
        M = np.zeros((16, 16), dtype=float)
        M[0:12, 0:12] = Ac
        M[0:12, 12:16] = Bc
        if HAS_SCIPY:
            expM = scipy.linalg.expm(M * dt)
        else:
            term = np.eye(16, dtype=float)
            expM = np.eye(16, dtype=float)
            M_dt = M * dt
            for k in range(1, 15):
                term = term @ M_dt / k
                expM += term
                if np.max(np.abs(term)) < 1e-15:
                    break
        A = expM[0:12, 0:12]
        B = expM[0:12, 12:16]
        return A, B

    def extract_state_deviation(
        self,
        p: np.ndarray,
        v: np.ndarray,
        R: np.ndarray,
        omega: np.ndarray,
        p_ref: np.ndarray
    ) -> np.ndarray:
        """
        Builds x_k = [p_k - pref; v_k; Log(R_k); ω_k] ∈ R^12
        """
        dp = np.asarray(p, dtype=float) - np.asarray(p_ref, dtype=float)
        dv = np.asarray(v, dtype=float)
        dtheta = so3_log(R)
        domega = np.asarray(omega, dtype=float)
        return np.concatenate([dp, dv, dtheta, domega])

    def extract_input_deviation(self, T: float, tau: np.ndarray) -> np.ndarray:
        """
        Builds u_k = [T_k - mg0; τ_k] ∈ R^4
        """
        dT = float(T) - self.mg0
        dtau = np.asarray(tau, dtype=float)
        return np.concatenate([[dT], dtau])

    def add_sample(self, x_k: np.ndarray, u_k: np.ndarray):
        """
        Appends (x_k, u_k) to rolling identification snapshot window.
        """
        self.x_history.append(np.array(x_k, dtype=float).copy())
        self.u_history.append(np.array(u_k, dtype=float).copy())
        self.samples_collected += 1

    def fit(self, regularization: float = 1e-4) -> bool:
        """
        Performs DMDc system identification:
        [A B] = X+ * [X; U]†
        """
        n_samples = len(self.x_history)
        if n_samples < self.min_fit_samples:
            return False

        # Assemble snapshots: X, X+ ∈ R^(12 x (n_samples-1)), U ∈ R^(4 x (n_samples-1))
        X = np.column_stack(list(self.x_history)[:-1])
        X_plus = np.column_stack(list(self.x_history)[1:])
        U = np.column_stack(list(self.u_history)[:-1])

        # Combined data matrix Omega ∈ R^(16 x (n_samples-1))
        Omega = np.vstack([X, U])

        try:
            # Regularized Moore-Penrose pseudo-inverse via SVD
            # [A B] = X+ * Omega^T * (Omega * Omega^T + reg * I)^(-1)
            reg_I = regularization * np.eye(Omega.shape[0])
            AB = X_plus @ Omega.T @ np.linalg.inv(Omega @ Omega.T + reg_I)

            A_learned = AB[:, 0:12]
            B_learned = AB[:, 12:16]

            # Validate numerical sanity & stability
            eigvals = np.linalg.eigvals(A_learned)
            max_eig = np.max(np.abs(eigvals))

            # Blend with physical prior if data variance is small (e.g. static hover)
            alpha = min(1.0, max(0.2, (n_samples - self.min_fit_samples) / float(self.max_samples)))
            if max_eig < 1.3:
                self.A = (1.0 - alpha) * self.A_phys + alpha * A_learned
                self.B = (1.0 - alpha) * self.B_phys + alpha * B_learned
                self.is_identified = True

                # Compute fit residual: ||X+ - (A*X + B*U)||_F / ||X+||_F
                pred = self.A @ X + self.B @ U
                norm_x = np.linalg.norm(X_plus)
                if norm_x > 1e-6:
                    self.identification_residual = float(np.linalg.norm(X_plus - pred) / norm_x)
                return True
        except Exception:
            pass

        return False

    def get_matrices(self):
        return self.A.copy(), self.B.copy()
