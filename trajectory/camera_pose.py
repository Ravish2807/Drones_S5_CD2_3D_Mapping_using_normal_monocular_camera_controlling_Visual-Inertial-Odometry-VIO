import numpy as np
from dataclasses import dataclass
from typing import Optional, List, Dict, Any, Tuple
from scipy.spatial.transform import Rotation, Slerp

from core.state import VIOState
from core.math_utils import rot_to_quat, quat_to_rot


def pose_to_matrix(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    """
    Constructs a 4x4 SE(3) transformation matrix from 3x3 rotation and 3D translation:
    T = [[R, p],
         [0, 1]]
    """
    R = np.asarray(R, dtype=np.float64)
    p = np.asarray(p, dtype=np.float64).flatten()
    assert R.shape == (3, 3), f"Expected R shape (3, 3), got {R.shape}"
    assert p.shape == (3,), f"Expected p shape (3,), got {p.shape}"
    T = np.eye(4, dtype=np.float64)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T


def matrix_to_pose(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Decomposes a 4x4 SE(3) transformation matrix into (R, p).
    """
    T = np.asarray(T, dtype=np.float64)
    assert T.shape == (4, 4), f"Expected T shape (4, 4), got {T.shape}"
    R = T[0:3, 0:3].copy()
    p = T[0:3, 3].copy()
    return R, p


@dataclass
class CameraExtrinsics:
    """
    Represents the spatial transformation T_BC from Camera frame to Body frame:
    - R_BC: 3x3 rotation matrix (transforms vector from Camera to Body)
    - p_BC: 3D translation vector (Camera origin expressed in Body coordinates)
    """
    R_BC: np.ndarray
    p_BC: np.ndarray

    def __post_init__(self):
        self.R_BC = np.asarray(self.R_BC, dtype=np.float64)
        self.p_BC = np.asarray(self.p_BC, dtype=np.float64).flatten()

    def to_matrix(self) -> np.ndarray:
        """Returns 4x4 matrix T_BC."""
        return pose_to_matrix(self.R_BC, self.p_BC)

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "CameraExtrinsics":
        extr = config.get("extrinsics", {})
        R_BC = np.array(extr.get("R_BC", [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]), dtype=np.float64)
        p_BC = np.array(extr.get("p_BC", [0.1, 0.0, 0.0]), dtype=np.float64)
        return cls(R_BC=R_BC, p_BC=p_BC)


@dataclass
class BodyPose:
    """
    Represents 6-DoF Drone/IMU Body Pose T_WB in the VIO World reference frame:
    - timestamp: epoch timestamp in seconds
    - p_WB: 3D position vector [x, y, z] in World frame (meters)
    - R_WB: 3x3 rotation matrix (World <- Body)
    - v_WB: optional 3D linear velocity in World frame (m/s)
    """
    timestamp: float
    p_WB: np.ndarray
    R_WB: np.ndarray
    v_WB: Optional[np.ndarray] = None
    bias_accel: Optional[np.ndarray] = None
    bias_gyro: Optional[np.ndarray] = None
    covariance_diag: Optional[np.ndarray] = None

    def __post_init__(self):
        self.p_WB = np.asarray(self.p_WB, dtype=np.float64).flatten()
        self.R_WB = np.asarray(self.R_WB, dtype=np.float64)

    def to_matrix(self) -> np.ndarray:
        """
        Step 2.3: Construct body pose matrix T_WB = [[R_WB, p_WB], [0, 1]]
        """
        return pose_to_matrix(self.R_WB, self.p_WB)

    @property
    def q_xyzw(self) -> np.ndarray:
        """Scalar-last quaternion [qx, qy, qz, qw]."""
        r = Rotation.from_matrix(self.R_WB)
        return r.as_quat()

    @property
    def q_wxyz(self) -> np.ndarray:
        """Hamilton scalar-first quaternion [qw, qx, qy, qz]."""
        return rot_to_quat(self.R_WB)

    @classmethod
    def from_vio_state(cls, state: VIOState) -> "BodyPose":
        """Instantiates BodyPose directly from MSCKF VIOState snapshot."""
        return cls(
            timestamp=state.timestamp,
            p_WB=state.position.copy(),
            R_WB=state.orientation_R.copy(),
            v_WB=state.velocity.copy(),
            bias_accel=state.bias_accel.copy() if state.bias_accel is not None else None,
            bias_gyro=state.bias_gyro.copy() if state.bias_gyro is not None else None,
            covariance_diag=state.covariance_diag.copy() if state.covariance_diag is not None else None,
        )


@dataclass
class CameraPose:
    """
    Represents 6-DoF Camera Pose T_WC in the VIO World reference frame:
    - timestamp: capture timestamp in seconds
    - p_WC: 3D camera focal center [x, y, z] in World frame (meters)
    - R_WC: 3x3 rotation matrix (World <- Camera)
    - frame_idx: optional camera frame index
    """
    timestamp: float
    p_WC: np.ndarray
    R_WC: np.ndarray
    frame_idx: Optional[int] = None
    source: str = "estimated" # 'estimated' or 'ground_truth'

    def __post_init__(self):
        self.p_WC = np.asarray(self.p_WC, dtype=np.float64).flatten()
        self.R_WC = np.asarray(self.R_WC, dtype=np.float64)

    def to_matrix(self) -> np.ndarray:
        """Returns 4x4 matrix T_WC."""
        return pose_to_matrix(self.R_WC, self.p_WC)

    @property
    def q_xyzw(self) -> np.ndarray:
        """Scalar-last quaternion [qx, qy, qz, qw] (TUM / ROS standard)."""
        r = Rotation.from_matrix(self.R_WC)
        return r.as_quat()

    @property
    def q_wxyz(self) -> np.ndarray:
        """Hamilton scalar-first quaternion [qw, qx, qy, qz]."""
        return rot_to_quat(self.R_WC)

    def to_tum_line(self) -> str:
        """Returns TUM format line: 'timestamp tx ty tz qx qy qz qw'."""
        tx, ty, tz = self.p_WC
        qx, qy, qz, qw = self.q_xyzw
        return f"{self.timestamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}"

    def to_csv_row(self) -> List[float]:
        """Returns [timestamp, tx, ty, tz, qx, qy, qz, qw]."""
        tx, ty, tz = self.p_WC
        qx, qy, qz, qw = self.q_xyzw
        return [float(self.timestamp), float(tx), float(ty), float(tz),
                float(qx), float(qy), float(qz), float(qw)]


def transform_body_to_camera_pose(body_pose: BodyPose, extrinsics: CameraExtrinsics, frame_idx: Optional[int] = None) -> CameraPose:
    """
    Step 2.4: Convert Body Pose (T_WB) to Camera Pose (T_WC):
    
    Mathematical Derivation:
    T_WC = T_WB * T_BC
    
    In block matrix form:
    [[R_WC, p_WC],    [[R_WB, p_WB],    [[R_BC, p_BC],
     [ 0  ,   1 ]]  =  [ 0  ,   1 ]]  *  [ 0  ,   1 ]]
                    = [[R_WB * R_BC,  R_WB * p_BC + p_WB],
                       [     0     ,             1        ]]
                       
    Therefore:
    R_WC = R_WB @ R_BC
    p_WC = p_WB + R_WB @ p_BC
    """
    p_WC = body_pose.p_WB + body_pose.R_WB @ extrinsics.p_BC
    R_WC = body_pose.R_WB @ extrinsics.R_BC

    return CameraPose(
        timestamp=body_pose.timestamp,
        p_WC=p_WC,
        R_WC=R_WC,
        frame_idx=frame_idx,
        source="estimated"
    )


class CameraTrajectory:
    """
    Ordered sequence of CameraPoses along with trajectory analytics,
    interpolation, and timestamp synchronization validation.
    """
    def __init__(self, poses: Optional[List[CameraPose]] = None, name: str = "camera_trajectory"):
        self.poses: List[CameraPose] = sorted(poses, key=lambda p: p.timestamp) if poses else []
        self.name = name

    def __len__(self) -> int:
        return len(self.poses)

    def __iter__(self):
        return iter(self.poses)

    def __getitem__(self, idx):
        return self.poses[idx]

    def append(self, pose: CameraPose):
        self.poses.append(pose)

    @property
    def timestamps(self) -> np.ndarray:
        return np.array([p.timestamp for p in self.poses], dtype=np.float64)

    @property
    def positions(self) -> np.ndarray:
        """Returns (N, 3) matrix of positions."""
        if len(self.poses) == 0:
            return np.empty((0, 3), dtype=np.float64)
        return np.array([p.p_WC for p in self.poses], dtype=np.float64)

    @property
    def rotations(self) -> np.ndarray:
        """Returns (N, 3, 3) matrix of rotation matrices."""
        if len(self.poses) == 0:
            return np.empty((0, 3, 3), dtype=np.float64)
        return np.array([p.R_WC for p in self.poses], dtype=np.float64)

    @property
    def quaternions_xyzw(self) -> np.ndarray:
        """Returns (N, 4) matrix of scalar-last quaternions [qx, qy, qz, qw]."""
        if len(self.poses) == 0:
            return np.empty((0, 4), dtype=np.float64)
        return np.array([p.q_xyzw for p in self.poses], dtype=np.float64)

    def interpolate_pose(self, t: float) -> Optional[CameraPose]:
        """
        Interpolates camera pose at arbitrary timestamp t using:
        - Linear interpolation for 3D translation
        - SLERP for SO(3) rotation
        """
        if len(self.poses) == 0:
            return None
        ts = self.timestamps
        if t < ts[0] or t > ts[-1]:
            return None

        idx = np.searchsorted(ts, t)
        if idx == 0:
            return self.poses[0]
        if ts[idx] == t:
            return self.poses[idx]

        t0, t1 = ts[idx - 1], ts[idx]
        alpha = (t - t0) / (t1 - t0)

        p0, p1 = self.poses[idx - 1].p_WC, self.poses[idx].p_WC
        p_interp = (1.0 - alpha) * p0 + alpha * p1

        r0 = Rotation.from_matrix(self.poses[idx - 1].R_WC)
        r1 = Rotation.from_matrix(self.poses[idx].R_WC)
        slerp = Slerp([t0, t1], Rotation.concatenate([r0, r1]))
        R_interp = slerp([t]).as_matrix()[0]

        return CameraPose(timestamp=t, p_WC=p_interp, R_WC=R_interp)

    def validate_timestamp_alignment(self, image_timestamps: List[float], max_allowed_dt: float = 1e-4) -> Dict[str, Any]:
        """
        Step 2.9: Validates 1-to-1 timestamp alignment between image frames and camera poses.
        Logs image_time, pose_time, and delta_t = |pose_time - image_time|.
        """
        records = []
        all_dts = []
        matched_count = 0

        pose_ts = self.timestamps

        for idx, img_t in enumerate(image_timestamps):
            if len(pose_ts) == 0:
                continue
            closest_idx = int(np.argmin(np.abs(pose_ts - img_t)))
            closest_pose_t = pose_ts[closest_idx]
            dt = abs(closest_pose_t - img_t)
            all_dts.append(dt)

            is_aligned = dt <= max_allowed_dt
            if is_aligned:
                matched_count += 1

            records.append({
                "frame_idx": idx,
                "image_time": float(img_t),
                "pose_time": float(closest_pose_t),
                "delta_t_sec": float(dt),
                "delta_t_ms": float(dt * 1000.0),
                "aligned": is_aligned
            })

        all_dts_arr = np.array(all_dts) if len(all_dts) > 0 else np.array([0.0])
        report = {
            "total_frames": len(image_timestamps),
            "matched_frames": matched_count,
            "match_ratio": float(matched_count / len(image_timestamps)) if len(image_timestamps) > 0 else 0.0,
            "max_delta_t_sec": float(np.max(all_dts_arr)),
            "mean_delta_t_sec": float(np.mean(all_dts_arr)),
            "is_strictly_synchronized": bool(np.max(all_dts_arr) <= max_allowed_dt),
            "records": records
        }
        return report
