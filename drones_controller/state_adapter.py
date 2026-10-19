from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class DroneState:
    """Standard internal representation of drone state in ENU frame."""
    position: np.ndarray          # [x, y, z] in meters
    velocity: np.ndarray          # [vx, vy, vz] in m/s
    rotation: np.ndarray          # 3x3 rotation matrix R in SO(3)
    quaternion: np.ndarray        # [qx, qy, qz, qw]
    angular_velocity: np.ndarray  # [wx, wy, wz] body rates in rad/s
    timestamp: float              # timestamp in seconds


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """
    ROS quaternion (x, y, z, w) -> Rotation Matrix R in SO(3)
    """
    q = np.array([qx, qy, qz, qw], dtype=float)
    norm = np.linalg.norm(q)

    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError("Invalid quaternion")

    q /= norm
    x, y, z, w = q

    R = np.array([
        [
            1.0 - 2.0 * (y*y + z*z),
            2.0 * (x*y - z*w),
            2.0 * (x*z + y*w)
        ],
        [
            2.0 * (x*y + z*w),
            1.0 - 2.0 * (x*x + z*z),
            2.0 * (y*z - x*w)
        ],
        [
            2.0 * (x*z - y*w),
            2.0 * (y*z + x*w),
            1.0 - 2.0 * (x*x + y*y)
        ]
    ], dtype=float)

    return R


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """
    Convert SO(3) rotation matrix to ROS quaternion [x, y, z, w].
    """
    trace = np.trace(R)

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw], dtype=float)
    q /= np.linalg.norm(q)
    return q


def vee(S: np.ndarray) -> np.ndarray:
    """
    Inverse of the hat/skew-symmetric operator.
             [ 0 -z  y ]
    S =      [ z  0 -x ]
             [-y  x  0 ]
    vee(S) = [x, y, z]
    """
    return np.array([
        S[2, 1],
        S[0, 2],
        S[1, 0]
    ], dtype=float)


def hat(v: np.ndarray) -> np.ndarray:
    """
    Vector [x, y, z] -> skew-symmetric 3x3 matrix.
    """
    x, y, z = v
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0]
    ], dtype=float)


def validate_rotation_matrix(R: np.ndarray, tol: float = 1e-3) -> bool:
    """Check if matrix is orthogonal with determinant 1."""
    if R is None or R.shape != (3, 3) or not np.all(np.isfinite(R)):
        return False
    orthogonality_error = np.linalg.norm(R.T @ R - np.eye(3))
    determinant_error = abs(np.linalg.det(R) - 1.0)
    return orthogonality_error < tol and determinant_error < tol


def so3_exp(v: np.ndarray) -> np.ndarray:
    """
    Exponential map: so(3) vector v -> SO(3) matrix R.
    Uses Rodrigues formula.
    """
    v = np.asarray(v, dtype=float)
    theta = np.linalg.norm(v)
    if theta < 1e-8:
        return np.eye(3) + hat(v)
    v_hat = hat(v)
    return np.eye(3) + (np.sin(theta) / theta) * v_hat + ((1.0 - np.cos(theta)) / (theta**2)) * (v_hat @ v_hat)


def so3_log(R: np.ndarray) -> np.ndarray:
    """
    Logarithmic map: SO(3) matrix R -> so(3) vector v (rotation vector).
    """
    R = np.asarray(R, dtype=float)
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(tr)
    if theta < 1e-8:
        return 0.5 * vee(R - R.T)
    return (theta / (2.0 * np.sin(theta))) * vee(R - R.T)

