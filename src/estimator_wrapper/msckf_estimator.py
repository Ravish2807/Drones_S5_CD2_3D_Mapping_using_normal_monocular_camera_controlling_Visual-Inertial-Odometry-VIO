import numpy as np
import time
from typing import Dict, List, Tuple, Optional, Any
from .math_utils import skew_symmetric, exp_so3, rot_to_quat, quat_to_rot
from .feature_tracker import FeatureTracker, FeatureTrack
from .imu_propagator import IMUPropagator


class CameraClone:
    """Cloned camera pose in the sliding window."""
    def __init__(self, clone_id: int, timestamp: float, p_WB: np.ndarray, R_WB: np.ndarray,
                 p_BC: np.ndarray, R_BC: np.ndarray):
        self.clone_id = clone_id
        self.timestamp = timestamp
        self.p_WB = p_WB.copy()
        self.R_WB = R_WB.copy()
        # Camera in world
        self.p_WC = self.R_WB @ p_BC + self.p_WB
        self.R_WC = self.R_WB @ R_BC
        self.R_CW = self.R_WC.T


class MSCKFEstimator:
    """
    Multi-State Constraint Kalman Filter (MSCKF) visual-inertial odometry estimator.
    Fuses continuous 6-DoF IMU measurements with tracked visual camera features.
    """

    def __init__(self, config: Dict[str, Any]):
        self.cfg = config
        est_cfg = config.get("estimator", {})
        ext_cfg = config.get("extrinsics", {})
        cam_cfg = config.get("camera", {})

        self.max_clones = est_cfg.get("max_clones", 12)
        self.min_track_length = est_cfg.get("min_track_length", 3)
        self.init_time = est_cfg.get("init_time_seconds", 1.0)
        self.gravity = np.array(est_cfg.get("gravity", [0.0, 0.0, -9.81]), dtype=np.float64)

        # Extrinsics
        self.R_BC = np.array(ext_cfg.get("R_BC", [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]), dtype=np.float64)
        self.p_BC = np.array(ext_cfg.get("p_BC", [0.10, 0.0, 0.0]), dtype=np.float64)

        # Camera intrinsics
        intrinsics = cam_cfg.get("intrinsics", {})
        self.fx = intrinsics.get("fx", 400.0)
        self.fy = intrinsics.get("fy", 400.0)
        self.cx = intrinsics.get("cx", 320.0)
        self.cy = intrinsics.get("cy", 240.0)
        self.feature_cov = 1.5**2 # feature pixel variance

        # Submodules
        self.propagator = IMUPropagator(config)
        self.tracker = FeatureTracker(config)

        # State initialization
        # IMU state: [p(3), v(3), R(3x3), ba(3), bg(3)]
        self.p_WB = np.zeros(3, dtype=np.float64)
        self.v_WB = np.zeros(3, dtype=np.float64)
        self.R_WB = np.eye(3, dtype=np.float64)
        self.b_a = np.zeros(3, dtype=np.float64)
        self.b_g = np.zeros(3, dtype=np.float64)

        # Covariance matrix P: starts with IMU error state (15x15)
        # Order: [delta_p(3), delta_v(3), delta_theta(3), delta_ba(3), delta_bg(3)]
        self.P = np.diag([
            0.01, 0.01, 0.01,         # p
            0.04, 0.04, 0.04,         # v
            0.005, 0.005, 0.005,      # theta
            0.01, 0.01, 0.01,         # ba
            0.001, 0.001, 0.001       # bg
        ])

        self.clones: Dict[int, CameraClone] = {} # {clone_id: CameraClone}
        self.clone_order: List[int] = []         # ordered list of clone IDs in covariance
        self.next_clone_id = 0

        self.last_imu_time: Optional[float] = None
        self.is_initialized = False
        self.init_imu_buffer: List[Dict[str, Any]] = []

        # History log for evaluation & plotting
        self.trajectory_history: List[Dict[str, Any]] = []
        self.timing_history: List[float] = []

    def initialize_from_imu_buffer(self, init_pos: Optional[np.ndarray] = None, init_R: Optional[np.ndarray] = None):
        """Initializes gravity alignment and gyroscope bias from static/quasi-static IMU buffer."""
        if len(self.init_imu_buffer) < 10:
            return

        accels = np.array([m["linear_accel"] for m in self.init_imu_buffer])
        gyros = np.array([m["angular_vel"] for m in self.init_imu_buffer])

        mean_acc = np.mean(accels, axis=0)
        mean_gyro = np.mean(gyros, axis=0)

        # Initialize gyro bias
        self.b_g = mean_gyro.copy()

        if init_R is not None:
            self.R_WB = init_R.copy()
        else:
            # Align z-axis to gravity: a_m = R^T * (-g) => z_body aligns with -a_m
            acc_norm = np.linalg.norm(mean_acc)
            if acc_norm > 1e-3:
                z_b = mean_acc / acc_norm
                x_b = np.array([1.0, 0.0, 0.0])
                if abs(np.dot(z_b, x_b)) > 0.9:
                    x_b = np.array([0.0, 1.0, 0.0])
                y_b = np.cross(z_b, x_b)
                y_b /= np.linalg.norm(y_b)
                x_b = np.cross(y_b, z_b)
                # R_BW = [x_b, y_b, z_b]^T
                self.R_WB = np.vstack([x_b, y_b, z_b]).T

        if init_pos is not None:
            self.p_WB = init_pos.copy()

        self.is_initialized = True

    def process_imu(self, timestamp: float, linear_accel: np.ndarray, angular_vel: np.ndarray):
        """Processes an incoming IMU sample (propagation step)."""
        if not self.is_initialized:
            self.init_imu_buffer.append({
                "timestamp": timestamp,
                "linear_accel": linear_accel,
                "angular_vel": angular_vel
            })
            if timestamp >= self.init_time:
                self.initialize_from_imu_buffer()
            self.last_imu_time = timestamp
            return

        if self.last_imu_time is None:
            self.last_imu_time = timestamp
            return

        dt = timestamp - self.last_imu_time
        if dt <= 0:
            return

        # 1. State propagation
        self.p_WB, self.v_WB, self.R_WB = self.propagator.propagate_state(
            self.p_WB, self.v_WB, self.R_WB, self.b_a, self.b_g, linear_accel, angular_vel, dt
        )

        # 2. Covariance propagation
        Phi_imu, Q_d = self.propagator.get_state_transition_and_noise(
            self.R_WB, self.b_a, self.b_g, linear_accel, angular_vel, dt
        )

        # Propagate IMU block
        self.P[:15, :15] = Phi_imu @ self.P[:15, :15] @ Phi_imu.T + Q_d

        # Propagate cross-terms between IMU state and cloned camera poses
        if len(self.clone_order) > 0:
            self.P[:15, 15:] = Phi_imu @ self.P[:15, 15:]
            self.P[15:, :15] = self.P[:15, 15:].T

        self.last_imu_time = timestamp

    def process_camera(self, timestamp: float, image: np.ndarray,
                       synthetic_features: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
        """
        Processes an incoming camera frame:
        1. Augments state with a new camera clone (T_WC).
        2. Tracks visual features.
        3. Identifies tracks ready for triangulation.
        4. Executes MSCKF measurement update via nullspace projection.
        5. Prunes old clones beyond max_clones.
        """
        t_start = time.perf_counter()

        if not self.is_initialized:
            return {"status": "uninitialized"}

        clone_id = self.next_clone_id
        self.next_clone_id += 1

        # 1. State Augmentation (Cloning)
        self._augment_state(clone_id, timestamp)

        # 2. Feature Tracking
        active_tracks, lost_tracks = self.tracker.track_frame(image, clone_id)

        # 3. Determine which tracks to update
        # Update tracks that were lost, or if window is full, tracks from oldest clone
        tracks_to_update = [t for t in lost_tracks if t.length >= self.min_track_length]

        if len(self.clone_order) >= self.max_clones:
            oldest_id = self.clone_order[0]
            for t in active_tracks:
                if oldest_id in t.observations and t.length >= self.min_track_length and t not in tracks_to_update:
                    tracks_to_update.append(t)

        # 4. MSCKF Update
        num_features_updated = 0
        if len(tracks_to_update) > 0:
            num_features_updated = self._update_features(tracks_to_update)

        # 5. Marginalize old clones if exceeding max_clones
        while len(self.clone_order) > self.max_clones:
            self._marginalize_oldest_clone()

        proc_time = time.perf_counter() - t_start
        self.timing_history.append(proc_time)

        # Record trajectory snapshot
        state_snapshot = {
            "timestamp": timestamp,
            "position": self.p_WB.copy(),
            "velocity": self.v_WB.copy(),
            "orientation_R": self.R_WB.copy(),
            "orientation_q": rot_to_quat(self.R_WB),
            "bias_accel": self.b_a.copy(),
            "bias_gyro": self.b_g.copy(),
            "active_features": len(active_tracks),
            "updated_features": num_features_updated,
            "clones_count": len(self.clone_order),
            "proc_time_sec": proc_time
        }
        self.trajectory_history.append(state_snapshot)

        return state_snapshot

    def _augment_state(self, clone_id: int, timestamp: float):
        """Adds current camera pose to the clone window and augments covariance."""
        clone = CameraClone(clone_id, timestamp, self.p_WB, self.R_WB, self.p_BC, self.R_BC)
        self.clones[clone_id] = clone
        self.clone_order.append(clone_id)

        # Jacobian of camera error pose [delta_p_C, delta_theta_C] w.r.t IMU state [delta_p, delta_v, delta_theta, ba, bg]
        # p_WC = p_WB + R_WB * p_BC => delta_p_WC = delta_p_WB - R_WB * [p_BC x] * delta_theta
        # R_WC = R_WB * R_BC       => delta_theta_WC = R_BC^T * delta_theta
        J_aug = np.zeros((6, 15), dtype=np.float64)
        J_aug[0:3, 0:3] = np.eye(3)
        J_aug[0:3, 6:9] = -self.R_WB @ skew_symmetric(self.p_BC)
        J_aug[3:6, 6:9] = self.R_BC.T

        # Augment covariance P: P_new = [P, P * J_aug^T; J_aug * P, J_aug * P_imu * J_aug^T]
        P_orig = self.P
        N_orig = P_orig.shape[0]

        P_aug_col = P_orig[:, :15] @ J_aug.T
        P_aug_bottom_right = J_aug @ P_orig[:15, :15] @ J_aug.T + np.eye(6) * 1e-6

        new_P = np.zeros((N_orig + 6, N_orig + 6), dtype=np.float64)
        new_P[:N_orig, :N_orig] = P_orig
        new_P[:N_orig, N_orig:] = P_aug_col
        new_P[N_orig:, :N_orig] = P_aug_col.T
        new_P[N_orig:, N_orig:] = P_aug_bottom_right

        self.P = new_P

    def _marginalize_oldest_clone(self):
        """Removes the oldest camera clone from the state and covariance."""
        if len(self.clone_order) == 0:
            return
        
        oldest_id = self.clone_order.pop(0)
        del self.clones[oldest_id]

        # Oldest clone occupies indices 15:21 in covariance
        indices_to_keep = [i for i in range(self.P.shape[0]) if i < 15 or i >= 21]
        self.P = self.P[np.ix_(indices_to_keep, indices_to_keep)]

    def _triangulate_landmark(self, track: FeatureTrack) -> Optional[np.ndarray]:
        """Triangulates 3D landmark in world coordinates from multiple camera clone observations."""
        obs = track.observations
        valid_clone_ids = [cid for cid in obs.keys() if cid in self.clones]
        if len(valid_clone_ids) < 2:
            return None

        # Parallax baseline check
        p_first = self.clones[valid_clone_ids[0]].p_WC
        p_last = self.clones[valid_clone_ids[-1]].p_WC
        if np.linalg.norm(p_first - p_last) < 0.03:
            return None

        A_dlt = []
        b = []
        for cid in valid_clone_ids:
            clone = self.clones[cid]
            R_CW = clone.R_CW
            p_WC = clone.p_WC
            _, _, xn, yn = obs[cid]

            A_dlt.append(xn * R_CW[2, :] - R_CW[0, :])
            b.append((xn * R_CW[2, :] - R_CW[0, :]) @ p_WC)

            A_dlt.append(yn * R_CW[2, :] - R_CW[1, :])
            b.append((yn * R_CW[2, :] - R_CW[1, :]) @ p_WC)

        A_dlt = np.array(A_dlt, dtype=np.float64)
        b = np.array(b, dtype=np.float64)

        try:
            P_W, _, _, _ = np.linalg.lstsq(A_dlt, b, rcond=None)

            # Check depth in all observing cameras
            for cid in valid_clone_ids:
                clone = self.clones[cid]
                P_C = clone.R_CW @ (P_W - clone.p_WC)
                if P_C[2] < 0.2 or P_C[2] > 60.0:
                    return None

            # Non-linear Gauss-Newton refinement (3 iterations)
            for _ in range(3):
                J_list = []
                res_list = []
                for cid in valid_clone_ids:
                    clone = self.clones[cid]
                    P_C = clone.R_CW @ (P_W - clone.p_WC)
                    Z = P_C[2]
                    z_hat = np.array([P_C[0] / Z, P_C[1] / Z])
                    _, _, xn, yn = obs[cid]
                    r = np.array([xn, yn]) - z_hat
                    J_proj = np.array([
                        [1.0 / Z, 0.0, -P_C[0] / (Z**2)],
                        [0.0, 1.0 / Z, -P_C[1] / (Z**2)]
                    ])
                    J_list.append(J_proj @ clone.R_CW)
                    res_list.append(r)
                
                J_mat = np.vstack(J_list)
                res_vec = np.concatenate(res_list)

                # Check max reprojection error
                if np.max(np.abs(res_vec)) > 0.03: # ~12 pixels
                    return None

                delta_P = np.linalg.lstsq(J_mat.T @ J_mat + 1e-4 * np.eye(3), J_mat.T @ res_vec, rcond=None)[0]
                P_W += delta_P
                if np.linalg.norm(delta_P) < 1e-3:
                    break

            return P_W
        except Exception:
            return None

    def _update_features(self, tracks: List[FeatureTrack]) -> int:
        """Computes MSCKF linear measurement model and applies EKF update."""
        H_rows = []
        r_rows = []

        total_state_dim = self.P.shape[0]

        # Sort tracks by length descending and cap to top 30
        tracks_sorted = sorted(tracks, key=lambda t: t.length, reverse=True)[:30]

        for track in tracks_sorted:
            P_W = self._triangulate_landmark(track)
            if P_W is None:
                continue

            valid_clone_ids = [cid for cid in track.observations.keys() if cid in self.clones]
            if len(valid_clone_ids) < 2:
                continue

            # Compute Jacobians for this feature
            H_x_feat = np.zeros((2 * len(valid_clone_ids), total_state_dim), dtype=np.float64)
            H_f_feat = np.zeros((2 * len(valid_clone_ids), 3), dtype=np.float64)
            r_feat = np.zeros(2 * len(valid_clone_ids), dtype=np.float64)

            for i, cid in enumerate(valid_clone_ids):
                clone = self.clones[cid]
                clone_idx_in_order = self.clone_order.index(cid)
                clone_cov_idx = 15 + clone_idx_in_order * 6

                P_C = clone.R_CW @ (P_W - clone.p_WC)
                Z = P_C[2]
                if Z < 0.1:
                    continue

                # Projected normalized coordinates
                z_hat = np.array([P_C[0] / Z, P_C[1] / Z])
                _, _, xn_obs, yn_obs = track.observations[cid]
                z_obs = np.array([xn_obs, yn_obs])

                r_feat[2*i : 2*i+2] = z_obs - z_hat

                # Jacobian of normalized coordinates w.r.t P_C
                J_proj = np.array([
                    [1.0 / Z, 0.0, -P_C[0] / (Z**2)],
                    [0.0, 1.0 / Z, -P_C[1] / (Z**2)]
                ])

                # Jacobian w.r.t P_W: d(P_C)/d(P_W) = R_CW
                H_f_feat[2*i : 2*i+2, :] = J_proj @ clone.R_CW

                # Jacobian w.r.t cloned pose [delta_p_C, delta_theta_C]
                J_pose = np.zeros((3, 6))
                J_pose[:, 0:3] = -clone.R_CW
                J_pose[:, 3:6] = skew_symmetric(P_C)

                H_x_feat[2*i : 2*i+2, clone_cov_idx : clone_cov_idx + 6] = J_proj @ J_pose

            # Nullspace projection of H_f_feat
            Q, R_qr = np.linalg.qr(H_f_feat, mode='complete')
            if Q.shape[1] > 3:
                V = Q[:, 3:].T
                H_o_feat = V @ H_x_feat
                r_o_feat = V @ r_feat

                H_rows.append(H_o_feat)
                r_rows.append(r_o_feat)

        if len(H_rows) == 0:
            return 0

        # Stack measurement equations
        H = np.vstack(H_rows)
        r = np.concatenate(r_rows)

        # Standard MSCKF QR Measurement Compression: H = Q_m * T_H
        if H.shape[0] > total_state_dim:
            Q_m, R_m_qr = np.linalg.qr(H, mode='reduced')
            H = R_m_qr
            r = Q_m.T @ r

        # Measurement noise covariance R_meas (isotropic)
        meas_noise_var = (1.0 / (self.fx**2)) * self.feature_cov
        R_m = np.eye(len(r), dtype=np.float64) * meas_noise_var

        # Kalman Gain: K = P * H^T * (H * P * H^T + R_m)^-1
        try:
            S = H @ self.P @ H.T + R_m
            K = self.P @ H.T @ np.linalg.inv(S)

            # Error state update
            delta_x = K @ r

            # Update IMU state
            self.p_WB += delta_x[0:3]
            self.v_WB += delta_x[3:6]
            self.R_WB = self.R_WB @ exp_so3(delta_x[6:9])
            self.b_a += delta_x[9:12]
            self.b_g += delta_x[12:15]

            # Update Clones
            for i, cid in enumerate(self.clone_order):
                idx = 15 + i * 6
                if idx + 6 <= len(delta_x):
                    self.clones[cid].p_WC += delta_x[idx : idx + 3]
                    self.clones[cid].R_WC = self.clones[cid].R_WC @ exp_so3(delta_x[idx + 3 : idx + 6])
                    self.clones[cid].R_CW = self.clones[cid].R_WC.T

            # Joseph form covariance update: P = (I - KH)P(I - KH)^T + K R_m K^T
            I_KH = np.eye(total_state_dim) - K @ H
            self.P = I_KH @ self.P @ I_KH.T + K @ R_m @ K.T
            # Ensure symmetry
            self.P = 0.5 * (self.P + self.P.T)

            return len(H_rows)
        except Exception:
            return 0
