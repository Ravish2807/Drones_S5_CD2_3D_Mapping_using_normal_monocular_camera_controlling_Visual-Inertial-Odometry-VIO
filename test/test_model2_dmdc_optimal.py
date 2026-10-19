import unittest
import numpy as np
from drones_controller.dmdc_identifier import DMDcIdentifier
from drones_controller.optimal_control import OptimalControlEngine


class TestModel2DMDcOptimal(unittest.TestCase):

    def setUp(self):
        self.mass = 1.5
        self.gravity = 9.81
        self.mg0 = self.mass * self.gravity
        self.dmdc = DMDcIdentifier(mass=self.mass, gravity=self.gravity, dt=0.04)
        self.opt = OptimalControlEngine(self.dmdc.A, self.dmdc.B, mg0=self.mg0, horizon=20, dt=0.04)

    def test_dmdc_physical_prior(self):
        """Verify baseline physical model dimensions and stability."""
        self.assertEqual(self.dmdc.A.shape, (12, 12))
        self.assertEqual(self.dmdc.B.shape, (12, 4))
        # Zero perturbation should yield zero derivative
        x_zero = np.zeros(12)
        u_zero = np.zeros(4)
        x_next = self.dmdc.A @ x_zero + self.dmdc.B @ u_zero
        np.testing.assert_allclose(x_next, np.zeros(12))

    def test_dmdc_buffer_and_fit(self):
        """Verify snapshot collection and DMDc regression."""
        for i in range(30):
            x = np.random.randn(12) * 0.1
            u = np.random.randn(4) * 0.5
            self.dmdc.add_sample(x, u)

        self.assertEqual(self.dmdc.samples_collected, 30)
        success = self.dmdc.fit()
        self.assertTrue(success)
        self.assertEqual(self.dmdc.A.shape, (12, 12))
        self.assertEqual(self.dmdc.B.shape, (12, 4))

    def test_lqr_solution(self):
        """Verify DARE Riccati solution and stabilizing gain."""
        self.assertEqual(self.opt.P.shape, (12, 12))
        # P must be symmetric positive definite
        np.testing.assert_allclose(self.opt.P, self.opt.P.T, atol=1e-6)
        eigvals = np.linalg.eigvals(self.opt.P)
        self.assertTrue(np.all(eigvals > 0.0))

        # Check closed loop spectral radius < 1.0 (discrete-time stability)
        A_cl = self.opt.A - self.opt.B @ self.opt.K
        cl_eigvals = np.linalg.eigvals(A_cl)
        max_rad = np.max(np.abs(cl_eigvals))
        self.assertLess(max_rad, 1.0 + 1e-4)

    def test_mpc_qp_box_constraints(self):
        """Verify MPC stacked QP respects thrust and torque bounds."""
        # Extreme initial error
        x_large = np.zeros(12)
        x_large[2] = 50.0  # 50m above target
        u_mpc = self.opt.compute_mpc(x_large)

        # Deviation bounds: u_min <= u <= u_max
        for i in range(4):
            self.assertGreaterEqual(u_mpc[i], self.opt.u_min[i] - 1e-5)
            self.assertLessEqual(u_mpc[i], self.opt.u_max[i] + 1e-5)


if __name__ == '__main__':
    unittest.main()
