import numpy as np
from scipy.spatial.transform import Rotation
from typing import Tuple, Optional


def interpolate_trajectory(query_times: np.ndarray, ref_times: np.ndarray, ref_positions: np.ndarray) -> np.ndarray:
    """
    Linearly interpolates 3D reference positions at query timestamps.
    """
    interp_pos = np.zeros((len(query_times), 3), dtype=np.float64)
    for dim in range(3):
        interp_pos[:, dim] = np.interp(query_times, ref_times, ref_positions[:, dim])
    return interp_pos


def align_trajectories_se3(est_pos: np.ndarray, gt_pos: np.ndarray, with_scale: bool = False) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray]:
    """
    Computes the optimal rigid SE(3) (or Sim(3)) transformation aligning est_pos to gt_pos using the Umeyama algorithm.
    Returns: (R_align, t_align, scale, aligned_est_pos)
    such that: aligned_est_pos = scale * (est_pos @ R_align.T) + t_align
    """
    assert len(est_pos) == len(gt_pos), "Trajectory lengths must match"
    N = len(est_pos)

    # 1. Compute centroids
    mu_est = np.mean(est_pos, axis=0)
    mu_gt = np.mean(gt_pos, axis=0)

    # 2. Center coordinates
    est_centered = est_pos - mu_est
    gt_centered = gt_pos - mu_gt

    # 3. Covariance matrix H
    H = est_centered.T @ gt_centered / N

    # 4. SVD of H
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Ensure right-handed coordinate system (det(R) = +1)
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    # 5. Scale calculation
    if with_scale:
        var_est = np.sum(np.var(est_pos, axis=0))
        scale = np.sum(S) / (var_est + 1e-12)
    else:
        scale = 1.0

    # 6. Translation
    t = mu_gt - scale * (R @ mu_est)

    # 7. Apply transform to estimated positions
    aligned_est = scale * (est_pos @ R.T) + t

    return R, t, scale, aligned_est
