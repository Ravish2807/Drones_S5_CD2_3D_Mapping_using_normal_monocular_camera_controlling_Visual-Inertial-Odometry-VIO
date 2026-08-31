import numpy as np
from typing import Dict, List, Optional, Tuple


def triangulate_linear_dlt(observations: Dict[int, Tuple[float, float]],
                           camera_poses: Dict[int, Tuple[np.ndarray, np.ndarray]],
                           min_depth: float = 0.2,
                           max_depth: float = 60.0) -> Optional[np.ndarray]:
    """
    Direct Linear Transform (DLT) multi-view 3D landmark triangulation.
    - observations: {clone_id: (xn_norm, yn_norm)}
    - camera_poses: {clone_id: (p_WC, R_CW)}
    """
    if len(observations) < 2:
        return None

    valid_cids = [cid for cid in observations if cid in camera_poses]
    if len(valid_cids) < 2:
        return None

    # Parallax baseline check
    p_first = camera_poses[valid_cids[0]][0]
    p_last = camera_poses[valid_cids[-1]][0]
    if np.linalg.norm(p_first - p_last) < 0.03:
        return None

    A_dlt = []
    b = []
    for cid in valid_cids:
        p_WC, R_CW = camera_poses[cid]
        xn, yn = observations[cid]

        A_dlt.append(xn * R_CW[2, :] - R_CW[0, :])
        b.append((xn * R_CW[2, :] - R_CW[0, :]) @ p_WC)

        A_dlt.append(yn * R_CW[2, :] - R_CW[1, :])
        b.append((yn * R_CW[2, :] - R_CW[1, :]) @ p_WC)

    A_dlt = np.array(A_dlt, dtype=np.float64)
    b = np.array(b, dtype=np.float64)

    try:
        P_W, _, _, _ = np.linalg.lstsq(A_dlt, b, rcond=None)

        # Cheirality and depth bound check
        for cid in valid_cids:
            p_WC, R_CW = camera_poses[cid]
            P_C = R_CW @ (P_W - p_WC)
            if P_C[2] < min_depth or P_C[2] > max_depth:
                return None

        # Non-linear Gauss-Newton optimization (3 iterations)
        for _ in range(3):
            J_list = []
            res_list = []
            for cid in valid_cids:
                p_WC, R_CW = camera_poses[cid]
                P_C = R_CW @ (P_W - p_WC)
                Z = P_C[2]
                z_hat = np.array([P_C[0] / Z, P_C[1] / Z])
                xn, yn = observations[cid]
                r = np.array([xn, yn]) - z_hat

                J_proj = np.array([
                    [1.0 / Z, 0.0, -P_C[0] / (Z**2)],
                    [0.0, 1.0 / Z, -P_C[1] / (Z**2)]
                ])
                J_list.append(J_proj @ R_CW)
                res_list.append(r)

            J_mat = np.vstack(J_list)
            res_vec = np.concatenate(res_list)

            # Reject outliers with excessive reprojection residual
            if np.max(np.abs(res_vec)) > 0.03: # ~12 pixels normalized
                return None

            delta_P = np.linalg.lstsq(J_mat.T @ J_mat + 1e-4 * np.eye(3), J_mat.T @ res_vec, rcond=None)[0]
            P_W += delta_P
            if np.linalg.norm(delta_P) < 1e-3:
                break

        return P_W
    except Exception:
        return None
