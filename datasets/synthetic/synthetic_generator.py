import os
import cv2
import numpy as np
from typing import Dict, Any, List, Tuple
from core.math_utils import exp_so3, rot_to_quat


class SyntheticDatasetGenerator:
    """
    Synthetic 6-DoF drone visual-inertial dataset generator.
    Produces synchronized camera images, 200 Hz IMU measurements, and GT trajectories.
    """

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        sim_cfg = config.get("simulation", {})
        cam_cfg = config.get("camera", {})
        imu_cfg = config.get("imu", {})

        self.duration = float(sim_cfg.get("duration_sec", 25.0))
        self.imu_freq = float(imu_cfg.get("rate_hz", 200.0))
        self.cam_freq = float(cam_cfg.get("fps", 20.0))

        # Camera intrinsics
        intrinsics = cam_cfg.get("intrinsics", {})
        self.fx = float(intrinsics.get("fx", 400.0))
        self.fy = float(intrinsics.get("fy", 400.0))
        self.cx = float(intrinsics.get("cx", 320.0))
        self.cy = float(intrinsics.get("cy", 240.0))
        self.width = int(cam_cfg.get("width", 640))
        self.height = int(cam_cfg.get("height", 480))

        # Extrinsics
        extrinsics = config.get("extrinsics", {})
        self.R_BC = np.array(extrinsics.get("R_BC", [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]), dtype=np.float64)
        self.p_BC = np.array(extrinsics.get("p_BC", [0.1, 0.0, 0.0]), dtype=np.float64)

        # World gravity
        self.gravity_W = np.array(config.get("world", {}).get("gravity", [0.0, 0.0, -9.81]), dtype=np.float64)

        # Generate landmark cloud
        self.landmarks_W = self._generate_landmarks()

    def _generate_landmarks(self) -> np.ndarray:
        """Generates random 3D landmark points in a cylinder around origin."""
        np.random.seed(42)
        n_pts = 500
        angles = np.random.uniform(0, 2 * np.pi, n_pts)
        radii = np.random.uniform(3.0, 9.0, n_pts)
        z = np.random.uniform(-1.0, 6.0, n_pts)
        x = radii * np.cos(angles)
        y = radii * np.sin(angles)
        return np.column_stack([x, y, z])

    def _trajectory_function(self, t: float, preset: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Returns (p_WB, v_WB, a_WB, R_WB, omega_B) at timestamp t."""
        if preset == "hover":
            p = np.array([0.0, 0.0, 2.0])
            v = np.zeros(3)
            a = np.zeros(3)
            R = np.eye(3)
            omega_B = np.zeros(3)
        elif preset == "straight":
            speed = 1.0
            p = np.array([speed * t, 0.0, 2.0])
            v = np.array([speed, 0.0, 0.0])
            a = np.zeros(3)
            R = np.eye(3)
            omega_B = np.zeros(3)
        elif preset == "orbit":
            R_orbit = 4.0
            omega_orbit = 2.0 * np.pi / 20.0
            yaw = omega_orbit * t + np.pi / 2.0

            p = np.array([R_orbit * np.cos(omega_orbit * t), R_orbit * np.sin(omega_orbit * t), 2.0 + 0.5 * np.sin(0.5 * t)])
            v = np.array([-R_orbit * omega_orbit * np.sin(omega_orbit * t), R_orbit * omega_orbit * np.cos(omega_orbit * t), 0.25 * np.cos(0.5 * t)])
            a = np.array([-R_orbit * (omega_orbit ** 2) * np.cos(omega_orbit * t), -R_orbit * (omega_orbit ** 2) * np.sin(omega_orbit * t), -0.125 * np.sin(0.5 * t)])

            cz, sz = np.cos(yaw), np.sin(yaw)
            R = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
            omega_B = np.array([0.0, 0.0, omega_orbit])
        elif preset == "multi_axis":
            wx, wy, wz = 0.4, 0.3, 0.5
            p = np.array([2.0 * np.sin(wx * t), 2.0 * np.cos(wy * t), 2.0 + 1.0 * np.sin(wz * t)])
            v = np.array([2.0 * wx * np.cos(wx * t), -2.0 * wy * np.sin(wy * t), wz * np.cos(wz * t)])
            a = np.array([-2.0 * (wx ** 2) * np.sin(wx * t), -2.0 * (wy ** 2) * np.cos(wy * t), -(wz ** 2) * np.sin(wz * t)])

            roll = 0.1 * np.sin(wx * t)
            pitch = 0.1 * np.cos(wy * t)
            yaw = 0.3 * t
            
            Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
            Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
            Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
            R = Rz @ Ry @ Rx
            omega_B = np.array([wx * 0.1 * np.cos(wx * t), -wy * 0.1 * np.sin(wy * t), 0.3])
        else: # Stress
            p, v, a, R, omega_B = self._trajectory_function(t, "multi_axis")

        return p, v, a, R, omega_B

    def generate_dataset(self, preset: str = "orbit", inject_dropout: bool = False) -> Dict[str, Any]:
        """Generates synchronized IMU measurements, Camera frames, and Ground Truth."""
        imu_times = np.arange(0, self.duration, 1.0 / self.imu_freq)
        cam_times = np.arange(0, self.duration, 1.0 / self.cam_freq)

        imu_measurements = []
        cam_frames = []
        gt_states = []

        imu_cfg = self.config.get("imu", {})
        accel_noise_std = float(imu_cfg.get("noise_accel", 0.005))
        gyro_noise_std = float(imu_cfg.get("noise_gyro", 0.001))
        rw_accel_std = float(imu_cfg.get("random_walk_accel", 0.0001))
        rw_gyro_std = float(imu_cfg.get("random_walk_gyro", 0.00001))

        b_a = np.zeros(3)
        b_g = np.zeros(3)
        dt_imu = 1.0 / self.imu_freq

        for t in imu_times:
            p_WB, v_WB, a_WB, R_WB, omega_B = self._trajectory_function(t, preset)

            # Random walk bias update
            b_a += np.random.normal(0, rw_accel_std * np.sqrt(dt_imu), 3)
            b_g += np.random.normal(0, rw_gyro_std * np.sqrt(dt_imu), 3)

            # Measured specific force: a_B = R_WB^T * (a_WB - g_W) + b_a + noise
            a_ideal_B = R_WB.T @ (a_WB - self.gravity_W)
            a_measured = a_ideal_B + b_a + np.random.normal(0, accel_noise_std, 3)
            w_measured = omega_B + b_g + np.random.normal(0, gyro_noise_std, 3)

            imu_measurements.append({
                "timestamp": float(t),
                "linear_accel": a_measured,
                "angular_vel": w_measured
            })

            gt_states.append({
                "timestamp": float(t),
                "position": p_WB.copy(),
                "velocity": v_WB.copy(),
                "R_WB": R_WB.copy(),
                "orientation_q": rot_to_quat(R_WB),
                "bias_accel": b_a.copy(),
                "bias_gyro": b_g.copy()
            })

        # Render camera frames
        for frame_idx, t in enumerate(cam_times):
            p_WB, _, _, R_WB, _ = self._trajectory_function(t, preset)

            R_WC = R_WB @ self.R_BC
            p_WC = R_WB @ self.p_BC + p_WB
            R_CW = R_WC.T

            img = np.zeros((self.height, self.width), dtype=np.uint8) + 40
            dropout_factor = 0.5 if (inject_dropout and 10.0 <= t <= 15.0) else 1.0

            for P_W in self.landmarks_W:
                if np.random.rand() > dropout_factor:
                    continue

                P_C = R_CW @ (P_W - p_WC)
                if P_C[2] > 0.4:
                    u = (self.fx * P_C[0] / P_C[2]) + self.cx + np.random.normal(0, 0.3)
                    v = (self.fy * P_C[1] / P_C[2]) + self.cy + np.random.normal(0, 0.3)

                    if 0 <= u < self.width and 0 <= v < self.height:
                        cv2.circle(img, (int(round(u)), int(round(v))), 3, 255, -1)
                        cv2.circle(img, (int(round(u)), int(round(v))), 6, 180, 1)

            cam_frames.append({
                "timestamp": float(t),
                "frame_idx": frame_idx,
                "image": img,
                "p_WC": p_WC,
                "R_WC": R_WC
            })

        return {
            "imu": imu_measurements,
            "camera": cam_frames,
            "ground_truth": gt_states,
            "landmarks": self.landmarks_W
        }
