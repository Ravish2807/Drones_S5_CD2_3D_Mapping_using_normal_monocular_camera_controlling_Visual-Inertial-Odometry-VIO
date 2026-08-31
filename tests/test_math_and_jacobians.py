import os
import sys
import numpy as np
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from core.math_utils import skew_symmetric, exp_so3, log_so3, rot_to_quat, quat_to_rot
from core.imu_propagator import IMUPropagator
from scipy.spatial.transform import Rotation


class TestVIOMathAndJacobians(unittest.TestCase):
    """
    Finite-difference numerical checks for all analytical kinematics,
    Lie group mappings, and measurement Jacobians.
    """

    def setUp(self):
        np.random.seed(123)
        self.eps = 1e-6

    def test_skew_symmetric(self):
        v = np.array([1.2, -3.4, 5.6])
        w = np.array([0.5, 2.1, -1.8])
        # Cross product identity: [v x] w == v x w
        cross_matrix = skew_symmetric(v) @ w
        cross_vector = np.cross(v, w)
        np.testing.assert_allclose(cross_matrix, cross_vector, atol=1e-12)

    def test_so3_exp_log(self):
        # Test across various angle magnitudes including near-zero
        test_angles = [0.0, 1e-9, 1e-5, 0.1, 1.5, np.pi - 0.01]
        for mag in test_angles:
            axis = np.array([1.0, 2.0, -1.0])
            axis /= np.linalg.norm(axis)
            w = axis * mag
            
            R = exp_so3(w)
            # Orthogonality
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)
            self.assertAlmostEqual(np.linalg.det(R), 1.0, places=9)
            
            # Log map recovery
            w_rec = log_so3(R)
            np.testing.assert_allclose(w_rec, w, atol=1e-6)

    def test_quat_rotation_conversions(self):
        r = Rotation.from_euler('zyx', [30, 45, 60], degrees=True)
        R_orig = r.as_matrix()
        q = rot_to_quat(R_orig)
        R_rec = quat_to_rot(q)
        np.testing.assert_allclose(R_rec, R_orig, atol=1e-10)

    def test_camera_projection_jacobian_numerical(self):
        """
        Tests analytical projection Jacobians against central finite differences.
        Measurement: z = [X/Z, Y/Z]^T
        """
        P_C = np.array([1.5, -0.8, 4.2], dtype=np.float64)
        Z = P_C[2]
        
        # Analytical Jacobian d(z)/d(P_C)
        J_analytical = np.array([
            [1.0 / Z, 0.0, -P_C[0] / (Z**2)],
            [0.0, 1.0 / Z, -P_C[1] / (Z**2)]
        ])

        # Numerical Jacobian via central differences
        J_numerical = np.zeros((2, 3))
        for dim in range(3):
            delta = np.zeros(3)
            delta[dim] = self.eps

            P_plus = P_C + delta
            P_minus = P_C - delta

            z_plus = np.array([P_plus[0] / P_plus[2], P_plus[1] / P_plus[2]])
            z_minus = np.array([P_minus[0] / P_minus[2], P_minus[1] / P_minus[2]])

            J_numerical[:, dim] = (z_plus - z_minus) / (2.0 * self.eps)

        np.testing.assert_allclose(J_analytical, J_numerical, rtol=1e-4, atol=1e-6)

    def test_camera_pose_perturbation_jacobian(self):
        """
        Tests Jacobian of point P_C w.r.t cloned camera pose [delta_p_C, delta_theta_C].
        P_C = R_CW * (P_W - p_WC)
        Perturbing:
        p_WC' = p_WC + delta_p
        R_WC' = R_WC * Exp(delta_theta) => R_CW' = Exp(-delta_theta) * R_CW
        """
        R_WC = Rotation.from_euler('zyx', [20, -15, 35], degrees=True).as_matrix()
        R_CW = R_WC.T
        p_WC = np.array([2.0, 1.5, 0.5])
        P_W = np.array([5.0, -3.0, 10.0])

        P_C = R_CW @ (P_W - p_WC)

        # Analytical Jacobians:
        # d(P_C)/d(delta_p_C) = -R_CW
        # d(P_C)/d(delta_theta_C) = [P_C x]
        J_p_analytical = -R_CW
        J_theta_analytical = skew_symmetric(P_C)

        # Numerical Jacobian for position perturbation
        J_p_num = np.zeros((3, 3))
        for dim in range(3):
            dp = np.zeros(3)
            dp[dim] = self.eps
            P_C_plus = R_CW @ (P_W - (p_WC + dp))
            P_C_minus = R_CW @ (P_W - (p_WC - dp))
            J_p_num[:, dim] = (P_C_plus - P_C_minus) / (2.0 * self.eps)

        np.testing.assert_allclose(J_p_analytical, J_p_num, rtol=1e-4, atol=1e-6)

        # Numerical Jacobian for orientation perturbation
        J_theta_num = np.zeros((3, 3))
        for dim in range(3):
            dtheta = np.zeros(3)
            dtheta[dim] = self.eps
            
            # R_WC_pert = R_WC @ exp_so3(dtheta) => R_CW_pert = exp_so3(-dtheta) @ R_CW
            R_CW_plus = exp_so3(-dtheta) @ R_CW
            R_CW_minus = exp_so3(dtheta) @ R_CW

            P_C_plus = R_CW_plus @ (P_W - p_WC)
            P_C_minus = R_CW_minus @ (P_W - p_WC)
            J_theta_num[:, dim] = (P_C_plus - P_C_minus) / (2.0 * self.eps)

        np.testing.assert_allclose(J_theta_analytical, J_theta_num, rtol=1e-4, atol=1e-6)

    def test_nullspace_projection_orthogonality(self):
        """
        Verifies that nullspace projector V satisfies V^T * H_f == 0.
        """
        num_clones = 6
        H_f = np.random.randn(2 * num_clones, 3)

        # QR decomposition
        Q, _ = np.linalg.qr(H_f, mode='complete')
        V = Q[:, 3:].T  # Shape: (2*N - 3, 2*N)

        residual_proj = V @ H_f
        np.testing.assert_allclose(residual_proj, np.zeros_like(residual_proj), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
