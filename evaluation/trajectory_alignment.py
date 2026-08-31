import numpy as np
from typing import Tuple


def align_trajectories_se3(est_pos: np.ndarray, gt_pos: np.ndarray, with_scale: bool = False) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray]:
    """
    Umeyama closed-form SE(3) / Sim(3) rigid trajectory alignment via SVD.
    Maps est_pos -> gt_pos: p_aligned = s * (est_pos @ R^T) + t
    """
    if len(est_pos) < 3 or len(gt_pos) < 3:
        return np.eye(3), np.zeros(3), 1.0, est_pos

    # Centroids
    mu_est = np.mean(est_pos, axis=0)
    mu_gt = np.mean(gt_pos, axis=0)

    est_centered = est_pos - mu_est
    gt_centered = gt_pos - mu_gt

    var_est = np.mean(np.sum(est_centered ** 2, axis=1))
    if var_est < 1e-7:
        return np.eye(3), mu_gt - mu_est, 1.0, est_pos + (mu_gt - mu_est)

    # Cross-covariance matrix H
    H = est_centered.T @ gt_centered / len(est_pos)

    U, D, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Reflection check
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    scale = 1.0
    if with_scale:
        scale = float(np.sum(D) / var_est)

    t = mu_gt - scale * (R @ mu_est)
    aligned_est_pos = scale * (est_pos @ R.T) + t

    return R, t, scale, aligned_est_pos


def interpolate_trajectory(query_times: np.ndarray, ref_times: np.ndarray, ref_values: np.ndarray) -> np.ndarray:
    """Linearly interpolates 3D trajectory at query timestamps."""
    out = np.zeros((len(query_times), ref_values.shape[1]), dtype=np.float64)
    for dim in range(ref_values.shape[1]):
        out[:, dim] = np.interp(query_times, ref_times, ref_values[:, dim])
    return out
