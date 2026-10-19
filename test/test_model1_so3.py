import unittest
import numpy as np
from drones_controller.state_adapter import so3_exp, so3_log, hat, vee
from drones_controller.error_state_ekf import ErrorStateEKF


class TestModel1SO3(unittest.TestCase):

    def test_so3_lie_algebra_roundtrip(self):
        """Verify Rodrigues exp and log round-trip on SO(3)."""
        test_vectors = [
            np.zeros(3),
            np.array([0.1, 0.0, 0.0]),
            np.array([0.0, -0.2, 0.0]),
            np.array([0.05, 0.1, -0.15]),
            np.array([np.pi / 4, 0.0, 0.0])
        ]
        for v in test_vectors:
            R = so3_exp(v)
            v_rec = so3_log(R)
            np.testing.assert_allclose(v_rec, v, atol=1e-10)

    def test_hat_vee_operator(self):
        """Verify hat and vee isomorphism."""
        w = np.array([1.2, -0.5, 3.1])
        W = hat(w)
        w_rec = vee(W)
        np.testing.assert_allclose(w_rec, w, atol=1e-12)

    def test_ekf_prediction_and_update(self):
        """Verify Error-State EKF predicts and updates without covariance explosion."""
        ekf = ErrorStateEKF(mass=1.5, gravity=9.81)
        ekf.initialize(p=np.zeros(3), v=np.zeros(3), R=np.eye(3), omega=np.zeros(3))
        initial_state = ekf.get_state()
        np.testing.assert_allclose(initial_state['p'], [0.0, 0.0, 0.0])

        # Predict with hover thrust T = mg = 1.5 * 9.81 = 14.715 N
        for _ in range(10):
            ekf.predict(T=14.715, tau=np.zeros(3), dt=0.02)

        # Drone hovering with thrust = mg should stay at ~0 vertical velocity
        state = ekf.get_state()
        self.assertAlmostEqual(state['v'][2], 0.0, places=2)

        # Measurement update: GPS position [1.0, 2.0, 3.0] with identity attitude and zero gyro
        ekf.update(z_p=np.array([1.0, 2.0, 3.0]), z_R=np.eye(3), z_omega=np.zeros(3))
        updated_state = ekf.get_state()
        self.assertGreater(updated_state['p'][0], 0.0)
        self.assertGreater(updated_state['p'][1], 0.0)
        self.assertGreater(updated_state['p'][2], 0.0)

        # Check covariance is symmetric positive definite
        P = ekf.P
        np.testing.assert_allclose(P, P.T, atol=1e-8)
        eigenvalues = np.linalg.eigvals(P)
        self.assertTrue(np.all(eigenvalues > 0.0))


if __name__ == '__main__':
    unittest.main()
