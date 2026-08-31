import numpy as np
from scipy.spatial.transform import Rotation


def skew_symmetric(v: np.ndarray) -> np.ndarray:
    """
    Computes the 3x3 skew-symmetric matrix for a 3D vector.
    [v x] = [[ 0,   -v[2],  v[1]],
             [ v[2],  0,   -v[0]],
             [-v[1],  v[0],  0  ]]
    """
    v = np.asarray(v).flatten()
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ], dtype=np.float64)


def exp_so3(w: np.ndarray) -> np.ndarray:
    """
    Exponential map from Lie algebra so(3) to Lie group SO(3) using Rodrigues formula.
    """
    theta = np.linalg.norm(w)
    if theta < 1e-8:
        return np.eye(3, dtype=np.float64) + skew_symmetric(w)
    
    axis = w / theta
    wx = skew_symmetric(axis)
    return np.eye(3, dtype=np.float64) + np.sin(theta) * wx + (1.0 - np.cos(theta)) * (wx @ wx)


def log_so3(R: np.ndarray) -> np.ndarray:
    """
    Logarithmic map from SO(3) to so(3).
    """
    trace = np.trace(R)
    cos_theta = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if theta < 1e-8:
        return np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) * 0.5
    
    return (theta / (2.0 * np.sin(theta))) * np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ])


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """
    Converts 3x3 rotation matrix to quaternion [w, x, y, z].
    """
    r = Rotation.from_matrix(R)
    xyzw = r.as_quat() # [x, y, z, w]
    return np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]], dtype=np.float64)


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """
    Converts quaternion [w, x, y, z] to 3x3 rotation matrix.
    """
    q_xyzw = np.array([q[1], q[2], q[3], q[0]], dtype=np.float64)
    return Rotation.from_quat(q_xyzw).as_matrix()


def normalize_quat(q: np.ndarray) -> np.ndarray:
    """
    Normalizes a quaternion [w, x, y, z].
    """
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm
