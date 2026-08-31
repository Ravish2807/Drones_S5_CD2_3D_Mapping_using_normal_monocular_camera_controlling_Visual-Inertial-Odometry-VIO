import numpy as np
import cv2
from typing import Dict, List, Tuple, Generator, Any, Optional


class SyntheticDatasetGenerator:
    """
    Generates synthetic visual-inertial drone trajectories, 3D landmarks,
    and ground-truth sensor measurements (camera projections & 6-DoF IMU).
    """

    def __init__(self, config: Dict[str, Any]):
        self.cfg = config
        self.cam_cfg = config.get("camera", {})
        self.imu_cfg = config.get("imu", {})
        self.sim_cfg = config.get("simulation", {})
        self.ext_cfg = config.get("extrinsics", {})

        self.fps = self.cam_cfg.get("fps", 20.0)
        self.imu_rate = self.imu_cfg.get("rate_hz", 200.0)
        self.duration = self.sim_cfg.get("duration_sec", 25.0)

        # Camera intrinsics
        intrinsics = self.cam_cfg.get("intrinsics", {})
        self.fx = intrinsics.get("fx", 400.0)
        self.fy = intrinsics.get("fy", 400.0)
        self.cx = intrinsics.get("cx", 320.0)
        self.cy = intrinsics.get("cy", 240.0)
        self.width = self.cam_cfg.get("width", 640)
        self.height = self.cam_cfg.get("height", 480)
        self.K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float64)

        # Extrinsics (Camera +Z forward aligns with +X body)
        self.R_BC = np.array(self.ext_cfg.get("R_BC", [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]), dtype=np.float64)
        self.p_BC = np.array(self.ext_cfg.get("p_BC", [0.10, 0.0, 0.0]), dtype=np.float64)

        # Noise parameters
        self.accel_noise_density = self.imu_cfg.get("noise_accel", 0.005)
        self.gyro_noise_density = self.imu_cfg.get("noise_gyro", 0.001)
        self.accel_rw = self.imu_cfg.get("random_walk_accel", 0.0001)
        self.gyro_rw = self.imu_cfg.get("random_walk_gyro", 0.00001)
        self.gravity_W = np.array(self.cfg.get("estimator", {}).get("gravity", [0.0, 0.0, -9.81]), dtype=np.float64)

        # Random seed for reproducibility
        np.random.seed(42)

        # Generate 3D landmark points
        self.num_landmarks = self.sim_cfg.get("num_landmarks", 500)
        self.landmarks_W = None

    def _generate_landmarks_for_preset(self, preset: str):
        if preset == "straight":
            bounds = [[-2.0, 24.0], [-6.0, 6.0], [-1.0, 6.0]]
        elif preset in ["orbit", "stress"]:
            bounds = [[-8.0, 8.0], [-8.0, 8.0], [-1.0, 6.0]]
        else:
            bounds = [[-7.0, 7.0], [-7.0, 7.0], [-1.0, 6.0]]

        np.random.seed(42)
        self.landmarks_W = np.random.uniform(
            low=[bounds[0][0], bounds[1][0], bounds[2][0]],
            high=[bounds[0][1], bounds[1][1], bounds[2][1]],
            size=(self.num_landmarks, 3)
        )

    def _trajectory_function(self, t: float, preset: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Returns ground truth (p_WB, v_WB, a_WB, R_WB, omega_WB_B) at time t.
        """
        if preset == "hover":
            # Near-stationary hover with subtle 1cm drift
            p = np.array([0.05 * np.sin(0.3 * t), 0.05 * np.cos(0.2 * t), 2.0 + 0.02 * np.sin(0.4 * t)])
            v = np.array([0.015 * np.cos(0.3 * t), -0.010 * np.sin(0.2 * t), 0.008 * np.cos(0.4 * t)])
            a = np.array([-0.0045 * np.sin(0.3 * t), -0.002 * np.cos(0.2 * t), -0.0032 * np.sin(0.4 * t)])
            # Orientation: small pitch/roll oscillation
            roll = 0.02 * np.sin(0.5 * t)
            pitch = 0.02 * np.cos(0.4 * t)
            yaw = 0.01 * t
            R = self._euler_to_R(roll, pitch, yaw)
            omega_B = np.array([0.01 * np.cos(0.5 * t), -0.008 * np.sin(0.4 * t), 0.01])

        elif preset == "straight":
            # Linear translation along X, rising along Z
            vx = 0.6
            vy = 0.1 * np.sin(0.4 * t)
            vz = 0.05
            p = np.array([vx * t, -0.25 * np.cos(0.4 * t) + 0.25, 1.5 + vz * t])
            v = np.array([vx, vy, vz])
            a = np.array([0.0, 0.04 * np.cos(0.4 * t), 0.0])
            yaw = 0.05 * np.sin(0.2 * t)
            pitch = 0.03 * np.sin(0.3 * t)
            roll = 0.02 * np.cos(0.3 * t)
            R = self._euler_to_R(roll, pitch, yaw)
            omega_B = np.array([0.0, 0.0, 0.01 * np.cos(0.2 * t)])

        elif preset == "orbit":
            # 360 degree circular orbit around center (0,0) looking inward
            radius = self.sim_cfg.get("orbit_radius", 4.0)
            alt = self.sim_cfg.get("orbit_altitude", 2.5)
            w_orbit = self.sim_cfg.get("orbit_speed_rad_s", 0.25)

            theta = w_orbit * t
            p = np.array([radius * np.cos(theta), radius * np.sin(theta), alt + 0.3 * np.sin(0.3 * t)])
            v = np.array([-radius * w_orbit * np.sin(theta), radius * w_orbit * np.cos(theta), 0.09 * np.cos(0.3 * t)])
            a = np.array([-radius * (w_orbit**2) * np.cos(theta), -radius * (w_orbit**2) * np.sin(theta), -0.027 * np.sin(0.3 * t)])

            # Drone heading points tangentially or towards center with yaw
            yaw = theta + np.pi / 2.0
            pitch = 0.05 * np.sin(0.5 * t)
            roll = 0.04 * np.cos(0.5 * t)
            R = self._euler_to_R(roll, pitch, yaw)
            omega_B = np.array([0.0, 0.0, w_orbit])

        elif preset == "multi_axis":
            # Dynamic multi-axis excitation
            p = np.array([
                2.5 * np.sin(0.4 * t),
                2.0 * np.sin(0.6 * t),
                2.0 + 0.8 * np.sin(0.3 * t)
            ])
            v = np.array([
                1.0 * np.cos(0.4 * t),
                1.2 * np.cos(0.6 * t),
                0.24 * np.cos(0.3 * t)
            ])
            a = np.array([
                -0.4 * np.sin(0.4 * t),
                -0.72 * np.sin(0.6 * t),
                -0.072 * np.sin(0.3 * t)
            ])
            roll = 0.15 * np.sin(0.8 * t)
            pitch = 0.15 * np.cos(0.7 * t)
            yaw = 0.3 * t
            R = self._euler_to_R(roll, pitch, yaw)
            omega_B = np.array([
                0.12 * np.cos(0.8 * t),
                -0.105 * np.sin(0.7 * t),
                0.3
            ])
        else: # stress preset
            # Fast aggressive orbit
            radius = 3.5
            w_orbit = 0.4
            theta = w_orbit * t
            p = np.array([radius * np.cos(theta), radius * np.sin(theta), 2.0 + 0.5 * np.sin(0.6 * t)])
            v = np.array([-radius * w_orbit * np.sin(theta), radius * w_orbit * np.cos(theta), 0.3 * np.cos(0.6 * t)])
            a = np.array([-radius * (w_orbit**2) * np.cos(theta), -radius * (w_orbit**2) * np.sin(theta), -0.18 * np.sin(0.6 * t)])
            yaw = theta + np.pi / 2.0
            R = self._euler_to_R(0.1 * np.sin(t), 0.1 * np.cos(t), yaw)
            omega_B = np.array([0.1 * np.cos(t), -0.1 * np.sin(t), w_orbit])

        return p, v, a, R, omega_B

    def _euler_to_R(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        # ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        cz, sz = np.cos(yaw), np.sin(yaw)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cx, sx = np.cos(roll), np.sin(roll)

        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        return Rz @ Ry @ Rx

    def generate_dataset(self, preset: Optional[str] = None, inject_dropout: bool = False) -> Dict[str, Any]:
        """
        Generates continuous time series for ground truth, IMU samples (200Hz), and Camera frames (20Hz).
        """
        chosen_preset = preset or self.sim_cfg.get("trajectory_preset", "orbit")
        self._generate_landmarks_for_preset(chosen_preset)
        dt_imu = 1.0 / self.imu_rate
        dt_cam = 1.0 / self.fps

        imu_times = np.arange(0, self.duration, dt_imu)
        cam_times = np.arange(0, self.duration, dt_cam)

        # Ground truth storage
        gt_states = []
        imu_measurements = []
        cam_frames = []

        # IMU biases initial & random walk simulation
        b_a = np.array([0.02, -0.015, 0.03], dtype=np.float64)
        b_g = np.array([0.002, 0.001, -0.003], dtype=np.float64)

        accel_noise_std = self.accel_noise_density * np.sqrt(self.imu_rate)
        gyro_noise_std = self.gyro_noise_density * np.sqrt(self.imu_rate)
        accel_rw_std = self.accel_rw * np.sqrt(dt_imu)
        gyro_rw_std = self.gyro_rw * np.sqrt(dt_imu)

        # Generate IMU time series
        for t in imu_times:
            p_WB, v_WB, a_WB, R_WB, omega_B = self._trajectory_function(t, chosen_preset)

            # Evolve biases
            b_a += np.random.normal(0, accel_rw_std, 3)
            b_g += np.random.normal(0, gyro_rw_std, 3)

            # IMU ideal measurement: a_m = R_WB^T * (a_WB - g_W) + b_a + noise
            a_ideal_B = R_WB.T @ (a_WB - self.gravity_W)
            a_measured = a_ideal_B + b_a + np.random.normal(0, accel_noise_std, 3)

            w_measured = omega_B + b_g + np.random.normal(0, gyro_noise_std, 3)

            imu_measurements.append({
                "timestamp": float(t),
                "linear_accel": a_measured,
                "angular_vel": w_measured
            })

            # Save GT
            gt_states.append({
                "timestamp": float(t),
                "position": p_WB.copy(),
                "velocity": v_WB.copy(),
                "R_WB": R_WB.copy(),
                "bias_accel": b_a.copy(),
                "bias_gyro": b_g.copy()
            })

        # Generate Camera frames and feature projections
        for frame_idx, t in enumerate(cam_times):
            p_WB, v_WB, a_WB, R_WB, _ = self._trajectory_function(t, chosen_preset)
            
            # Camera pose in World: T_WC = T_WB * T_BC
            R_WC = R_WB @ self.R_BC
            p_WC = R_WB @ self.p_BC + p_WB
            R_CW = R_WC.T

            # Project landmarks into camera frame
            visible_features = []
            img = np.zeros((self.height, self.width), dtype=np.uint8) + 40 # Dark grey background

            # Dropout simulation: occasionally drop features
            dropout_factor = 0.5 if (inject_dropout and 10.0 <= t <= 15.0) else 1.0

            for pt_id, P_W in enumerate(self.landmarks_W):
                if np.random.rand() > dropout_factor:
                    continue

                # Point in camera frame: P_C = R_CW * (P_W - p_WC)
                P_C = R_CW @ (P_W - p_WC)
                if P_C[2] > 0.4: # In front of camera
                    u = (self.fx * P_C[0] / P_C[2]) + self.cx + np.random.normal(0, 0.4)
                    v = (self.fy * P_C[1] / P_C[2]) + self.cy + np.random.normal(0, 0.4)

                    if 0 <= u < self.width and 0 <= v < self.height:
                        visible_features.append({
                            "id": pt_id,
                            "uv": np.array([u, v], dtype=np.float64),
                            "point_3d": P_W.copy()
                        })
                        # Render synthetic marker on image
                        cv2.circle(img, (int(round(u)), int(round(v))), 3, 255, -1)
                        # Add a small gradient patch around point for optical flow tracking
                        cv2.circle(img, (int(round(u)), int(round(v))), 6, 180, 1)

            cam_frames.append({
                "timestamp": float(t),
                "frame_idx": frame_idx,
                "image": img,
                "features": visible_features,
                "R_WC": R_WC,
                "p_WC": p_WC
            })

        return {
            "imu": imu_measurements,
            "camera": cam_frames,
            "ground_truth": gt_states,
            "landmarks": self.landmarks_W,
            "preset": chosen_preset
        }
