import time
import numpy as np
from typing import Dict, Any, List, Optional, Tuple

from .state import VIOState
from .math_utils import exp_so3, rot_to_quat, skew_symmetric
from .frames import FrameTransforms
from .imu_propagator import IMUPropagator
from .feature_tracker import FeatureTracker, FeatureTrack
from .triangulation import triangulate_linear_dlt


class CameraClone:
    """Represents a cloned camera pose in the sliding window."""
    def __init__(self, clone_id: int, timestamp: float,
                 p_WB: np.ndarray, R_WB: np.ndarray,
                 p_BC: np.ndarray, R_BC: np.ndarray):
        self.clone_id = clone_id
        self.timestamp = timestamp
        self.p_WC = R_WB @ p_BC + p_WB
        self.R_WC = R_WB @ R_BC
        self.R_CW = self.R_WC.T


class MSCKFEstimator:
    """
    Mathematical MSCKF Visual-Inertial Odometry Estimator.
    Decoupled from operating system, datasets, or messaging layers.
    """

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        est_cfg = config.get("estimator", {})
        cam_cfg = config.get("camera", {})

        # Sliding window parameters
        self.max_clones = int(est_cfg.get("max_clones", 10))
        self.min_track_length = int(est_cfg.get("min_track_length", 3))
        self.feature_cov = float(est_cfg.get("feature_cov", 1.0)) # pixels^2

        # Camera intrinsics
        intrinsics = cam_cfg.get("intrinsics", {})
        self.fx = float(intrinsics.get("fx", 400.0))
        self.fy = float(intrinsics.get("fy", 400.0))
        self.cx = float(intrinsics.get("cx", 320.0))
        self.cy = float(intrinsics.get("cy", 240.0))

        # Extrinsic calibration (Camera relative to Body)
        extrinsics = config.get("extrinsics", {})
        self.R_BC = np.array(extrinsics.get("R_BC", [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]), dtype=np.float64)
        self.p_BC = np.array(extrinsics.get("p_BC", [0.1, 0.0, 0.0]), dtype=np.float64)
        self.frames = FrameTransforms(self.R_BC, self.p_BC)

        # Core sub-modules
        self.propagator = IMUPropagator(config)
        self.tracker = FeatureTracker(config)

        # 6-DoF State Vector: p_WB (0:3), v_WB (3:6), R_WB (6:9), b_a (9:12), b_g (12:15)
        self.p_WB = np.zeros(3, dtype=np.float64)
        self.v_WB = np.zeros(3, dtype=np.float64)
        self.R_WB = np.eye(3, dtype=np.float64)
        self.b_a = np.zeros(3, dtype=np.float64)
        self.b_g = np.zeros(3, dtype=np.float64)

        # State error covariance P (15x15 initially)
        self.P = np.eye(15, dtype=np.float64)
        init_cov = est_cfg.get("initial_covariance", {})
        self.P[0:3, 0:3] *= float(init_cov.get("p", 1e-4))
        self.P[3:6, 3:6] *= float(init_cov.get("v", 1e-3))
        self.P[6:9, 6:9] *= float(init_cov.get("q", 1e-4))
        self.P[9:12, 9:12] *= float(init_cov.get("ba", 1e-2))
        self.P[12:15, 12:15] *= float(init_cov.get("bg", 1e-4))

        # Cloned camera poses: clone_id -> CameraClone
        self.clones: Dict[int, CameraClone] = {}
        self.clone_order: List[int] = []
        self.next_clone_id = 0

        # State tracking & buffers
        self.is_initialized = False
        self.last_imu_time: Optional[float] = None
        self.init_imu_buffer: List[Dict[str, Any]] = []
        self.init_time = float(config.get("initialization", {}).get("init_duration_sec", 0.5))

        self.current_state: Optional[VIOState] = None
        self.landmarks_map: Dict[int, np.ndarray] = {}

    def get_landmark_cloud(self) -> np.ndarray:
        """Returns array of triangulated 3D landmark points in world frame (N, 3)."""
        if len(self.landmarks_map) == 0:
            return np.empty((0, 3), dtype=np.float64)
        arr = np.array(list(self.landmarks_map.values()), dtype=np.float64)
        return arr[:, :3]

    def export_pointcloud_ply(self, filepath: str):
        """Exports triangulated 3D landmark point cloud with RGB colors to standard PLY format."""
        import os
        if len(self.landmarks_map) == 0:
            pts = np.empty((0, 6), dtype=np.float64)
        else:
            pts = np.array(list(self.landmarks_map.values()), dtype=np.float64)
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        with open(filepath, "w") as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(pts)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write("end_header\n")
            for p in pts:
                r = int(np.clip(p[3] if len(p) >= 6 else 200, 0, 255))
                g = int(np.clip(p[4] if len(p) >= 6 else 200, 0, 255))
                b = int(np.clip(p[5] if len(p) >= 6 else 200, 0, 255))
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {r} {g} {b}\n")
        print(f"Exported 3D Colored Point Cloud ({len(pts)} points) to {filepath}")

    def export_pointcloud_pcd(self, filepath: str):
        """Exports triangulated 3D landmark point cloud to standard PCD format."""
        import os
        pts = self.get_landmark_cloud()
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        with open(filepath, "w") as f:
            f.write("# .PCD v.7 - Point Cloud Data file format\n")
            f.write("VERSION .7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(pts)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(pts)}\n")
            f.write("DATA ascii\n")
            for p in pts:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
        print(f"Exported 3D Point Cloud PCD ({len(pts)} points) to {filepath}")

    def initialize_from_imu(self, init_pos: Optional[np.ndarray] = None, init_R: Optional[np.ndarray] = None):
        """Initializes estimator orientation and gravity alignment."""
        if len(self.init_imu_buffer) == 0 and init_R is None:
            return

        if len(self.init_imu_buffer) > 0:
            acc_samples = np.array([s["linear_accel"] for s in self.init_imu_buffer])
            mean_acc = np.mean(acc_samples, axis=0)
        else:
            mean_acc = np.array([0.0, 0.0, 9.81])

        if init_R is not None:
            self.R_WB = init_R.copy()
        else:
            acc_norm = np.linalg.norm(mean_acc)
            if acc_norm > 1e-3:
                z_b = mean_acc / acc_norm
                x_b = np.array([1.0, 0.0, 0.0])
                if abs(np.dot(z_b, x_b)) > 0.9:
                    x_b = np.array([0.0, 1.0, 0.0])
                y_b = np.cross(z_b, x_b)
                y_b /= np.linalg.norm(y_b)
                x_b = np.cross(y_b, z_b)
                self.R_WB = np.vstack([x_b, y_b, z_b]).T

        if init_pos is not None:
            self.p_WB = init_pos.copy()

        self.is_initialized = True

    def process_imu(self, timestamp: float, linear_accel: np.ndarray, angular_vel: np.ndarray):
        """Processes incoming IMU sample (propagation step)."""
        if not self.is_initialized:
            self.init_imu_buffer.append({
                "timestamp": timestamp,
                "linear_accel": linear_accel,
                "angular_vel": angular_vel
            })
            if timestamp >= self.init_time:
                self.initialize_from_imu()
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

        self.P[:15, :15] = Phi_imu @ self.P[:15, :15] @ Phi_imu.T + Q_d
        if len(self.clone_order) > 0:
            self.P[:15, 15:] = Phi_imu @ self.P[:15, 15:]
            self.P[15:, :15] = self.P[:15, 15:].T

        self.last_imu_time = timestamp

    def process_camera(self, timestamp: float, image: np.ndarray) -> Optional[VIOState]:
        """
        Processes incoming camera frame (measurement update step):
        1. Augments state with a new camera clone.
        2. Tracks visual features.
        3. Triangulates and nullspace-projects measurement constraints.
        4. Applies compressed Kalman filter update.
        5. Prunes oldest clones beyond sliding window limit.
        """
        t_start = time.perf_counter()

        if not self.is_initialized:
            return None

        # 1. Augment state with current camera clone
        clone_id = self.next_clone_id
        self.next_clone_id += 1
        self._augment_state(clone_id, timestamp)

        # 2. Track visual features
        active_tracks, lost_tracks = self.tracker.track_frame(image, clone_id)

        # 3. Identify tracks ready for MSCKF update
        update_tracks: List[FeatureTrack] = []
        for track in lost_tracks:
            if track.length >= self.min_track_length:
                update_tracks.append(track)

        # Force update oldest tracks if clone window is full
        if len(self.clone_order) > self.max_clones:
            oldest_clone_id = self.clone_order[0]
            for track in active_tracks:
                if oldest_clone_id in track.observations and track.length >= self.min_track_length:
                    if track not in update_tracks:
                        update_tracks.append(track)

        # 4. Perform MSCKF measurement update
        num_updated = 0
        if len(update_tracks) > 0:
            num_updated = self._update_features(update_tracks)

        # 5. Prune sliding window
        while len(self.clone_order) > self.max_clones:
            self._marginalize_oldest_clone()

        proc_time = time.perf_counter() - t_start

        # Package current state snapshot
        q_curr = rot_to_quat(self.R_WB)
        self.current_state = VIOState(
            timestamp=timestamp,
            position=self.p_WB.copy(),
            velocity=self.v_WB.copy(),
            orientation_R=self.R_WB.copy(),
            orientation_q=q_curr,
            bias_accel=self.b_a.copy(),
            bias_gyro=self.b_g.copy(),
            active_features=len(active_tracks),
            updated_features=num_updated,
            clones_count=len(self.clone_order),
            covariance_diag=np.diag(self.P[:15, :15]).copy(),
            quality_score=min(1.0, len(active_tracks) / 100.0),
            latency_sec=proc_time
        )

        return self.current_state

    def _augment_state(self, clone_id: int, timestamp: float):
        """Adds current camera pose to clone window and augments covariance."""
        clone = CameraClone(clone_id, timestamp, self.p_WB, self.R_WB, self.p_BC, self.R_BC)
        self.clones[clone_id] = clone
        self.clone_order.append(clone_id)

        # Jacobian of camera error pose [delta_p_C, delta_theta_C] w.r.t IMU state [delta_p, delta_v, delta_theta, ba, bg]
        J_aug = np.zeros((6, 15), dtype=np.float64)
        J_aug[0:3, 0:3] = np.eye(3)
        J_aug[0:3, 6:9] = -self.R_WB @ skew_symmetric(self.p_BC)
        J_aug[3:6, 6:9] = self.R_BC.T

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
        """Removes the oldest camera clone from sliding window."""
        if len(self.clone_order) == 0:
            return
        oldest_id = self.clone_order.pop(0)
        del self.clones[oldest_id]

        indices_to_keep = [i for i in range(self.P.shape[0]) if i < 15 or i >= 21]
        self.P = self.P[np.ix_(indices_to_keep, indices_to_keep)]

    def _update_features(self, tracks: List[FeatureTrack]) -> int:
        """Executes multi-state constraint measurement update."""
        total_state_dim = self.P.shape[0]
        H_rows = []
        r_rows = []

        # Sort tracks by length
        tracks_sorted = sorted(tracks, key=lambda t: t.length, reverse=True)[:30]

        for track in tracks_sorted:
            obs = {cid: (track.observations[cid][2], track.observations[cid][3])
                   for cid in track.observations if cid in self.clones}
            camera_poses = {cid: (self.clones[cid].p_WC, self.clones[cid].R_CW) for cid in obs}

            P_W = triangulate_linear_dlt(obs, camera_poses)
            if P_W is None:
                continue

            r, g, b = track.color
            self.landmarks_map[track.track_id] = np.array([P_W[0], P_W[1], P_W[2], r, g, b], dtype=np.float64)

            valid_clone_ids = list(obs.keys())
            if len(valid_clone_ids) < 2:
                continue

            H_x_feat = np.zeros((2 * len(valid_clone_ids), total_state_dim), dtype=np.float64)
            H_f_feat = np.zeros((2 * len(valid_clone_ids), 3), dtype=np.float64)
            r_feat = np.zeros(2 * len(valid_clone_ids), dtype=np.float64)

            for i, cid in enumerate(valid_clone_ids):
                clone = self.clones[cid]
                clone_idx = self.clone_order.index(cid)
                clone_cov_idx = 15 + clone_idx * 6

                P_C = clone.R_CW @ (P_W - clone.p_WC)
                Z = P_C[2]
                if Z < 0.1:
                    continue

                z_hat = np.array([P_C[0] / Z, P_C[1] / Z])
                z_obs = np.array(obs[cid])
                r_feat[2*i : 2*i+2] = z_obs - z_hat

                J_proj = np.array([
                    [1.0 / Z, 0.0, -P_C[0] / (Z**2)],
                    [0.0, 1.0 / Z, -P_C[1] / (Z**2)]
                ])

                H_f_feat[2*i : 2*i+2, :] = J_proj @ clone.R_CW

                J_pose = np.zeros((3, 6))
                J_pose[:, 0:3] = -clone.R_CW
                J_pose[:, 3:6] = skew_symmetric(P_C)

                H_x_feat[2*i : 2*i+2, clone_cov_idx : clone_cov_idx + 6] = J_proj @ J_pose

            # Nullspace projection
            Q, _ = np.linalg.qr(H_f_feat, mode='complete')
            if Q.shape[1] > 3:
                V = Q[:, 3:].T
                H_rows.append(V @ H_x_feat)
                r_rows.append(V @ r_feat)

        if len(H_rows) == 0:
            return 0

        H = np.vstack(H_rows)
        r = np.concatenate(r_rows)

        # QR measurement compression
        if H.shape[0] > total_state_dim:
            Q_m, R_m_qr = np.linalg.qr(H, mode='reduced')
            H = R_m_qr
            r = Q_m.T @ r

        meas_noise_var = (1.0 / (self.fx**2)) * self.feature_cov
        R_m = np.eye(len(r), dtype=np.float64) * meas_noise_var

        try:
            S = H @ self.P @ H.T + R_m
            K = self.P @ H.T @ np.linalg.inv(S)

            delta_x = K @ r

            # State correction injection
            self.p_WB += delta_x[0:3]
            self.v_WB += delta_x[3:6]
            self.R_WB = self.R_WB @ exp_so3(delta_x[6:9])
            self.b_a += delta_x[9:12]
            self.b_g += delta_x[12:15]

            for i, cid in enumerate(self.clone_order):
                idx = 15 + i * 6
                if idx + 6 <= len(delta_x):
                    self.clones[cid].p_WC += delta_x[idx : idx + 3]
                    self.clones[cid].R_WC = self.clones[cid].R_WC @ exp_so3(delta_x[idx + 3 : idx + 6])
                    self.clones[cid].R_CW = self.clones[cid].R_WC.T

            # Joseph form covariance update
            I_KH = np.eye(total_state_dim) - K @ H
            self.P = I_KH @ self.P @ I_KH.T + K @ R_m @ K.T
            self.P = 0.5 * (self.P + self.P.T)

            return len(H_rows)
        except Exception:
            return 0
