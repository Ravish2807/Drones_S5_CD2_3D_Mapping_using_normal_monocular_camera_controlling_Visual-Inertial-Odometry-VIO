import numpy as np

from .state_adapter import vee


class SO3Controller:

    def __init__(
        self,
        mass,
        gravity,
        inertia,
        kp,
        kv,
        kR,
        kOmega
    ):

        self.mass = float(mass)
        self.gravity = float(gravity)

        self.J = np.diag(np.array(inertia, dtype=float))

        self.Kp = np.diag(np.array(kp, dtype=float))
        self.Kv = np.diag(np.array(kv, dtype=float))

        self.KR = np.diag(np.array(kR, dtype=float))
        self.KOmega = np.diag(np.array(kOmega, dtype=float))

        self.e3 = np.array([0.0, 0.0, 1.0])

    def compute_position_errors(self, p, v, desired):

        p_d = desired["p_d"]
        v_d = desired["v_d"]

        e_p = p_d - p
        e_v = v_d - v

        return e_p, e_v

    def compute_desired_force(
        self,
        e_p,
        e_v,
        a_d
    ):
        """
        ENU-adapted translational controller.

        ArduPilot ROS 2 pose/twist are documented in local ENU.

        a_cmd = Kp*ep + Kv*ev + a_d

        F_d = m * (a_cmd + g*e3)

        This produces an upward force for hover in ENU.

        The uploaded controller notes use a different sign convention
        for the force equation, so this frame/sign adaptation must be
        validated experimentally.
        """

        a_cmd = (
            self.Kp @ e_p
            + self.Kv @ e_v
            + a_d
        )

        F_d = self.mass * (
            a_cmd + self.gravity * self.e3
        )

        return F_d

    def compute_desired_attitude(
        self,
        F_d,
        yaw_d
    ):

        norm_F = np.linalg.norm(F_d)

        if norm_F < 1e-6:

            return np.eye(3)

        b3 = F_d / norm_F

        yaw_heading = np.array([
            np.cos(yaw_d),
            np.sin(yaw_d),
            0.0
        ])

        b2 = np.cross(b3, yaw_heading)

        norm_b2 = np.linalg.norm(b2)

        if norm_b2 < 1e-6:

            yaw_d += 1e-3

            yaw_heading = np.array([
                np.cos(yaw_d),
                np.sin(yaw_d),
                0.0
            ])

            b2 = np.cross(b3, yaw_heading)

            norm_b2 = np.linalg.norm(b2)

        b2 /= norm_b2

        b1 = np.cross(b2, b3)

        b1 /= np.linalg.norm(b1)

        R_d = np.column_stack([
            b1,
            b2,
            b3
        ])

        return R_d

    def compute_attitude_error(
        self,
        R,
        R_d
    ):

        E = (
            R_d.T @ R
            -
            R.T @ R_d
        )

        e_R = 0.5 * vee(E)

        return e_R

    def compute_angular_velocity_error(
        self,
        R,
        R_d,
        omega,
        omega_d
    ):

        e_omega = (
            omega
            -
            R.T @ R_d @ omega_d
        )

        return e_omega

    def compute_moment(
        self,
        R,
        R_d,
        omega,
        omega_d,
        omega_dot_d,
        e_R,
        e_omega
    ):
        """
        Full SO(3) moment law from the controller architecture.

        M_d =
        -KR e_R
        -KOmega eOmega
        + omega x (J omega)
        -J( omega_hat R^T Rd omega_d
            - R^T Rd omega_dot_d )
        """

        relative_R = R.T @ R_d

        coriolis = np.cross(
            omega,
            self.J @ omega
        )

        omega_hat = np.array([
            [0.0, -omega[2], omega[1]],
            [omega[2], 0.0, -omega[0]],
            [-omega[1], omega[0], 0.0]
        ])

        feedforward = self.J @ (
            omega_hat @ relative_R @ omega_d
            -
            relative_R @ omega_dot_d
        )

        M_d = (
            -self.KR @ e_R
            -
            self.KOmega @ e_omega
            +
            coriolis
            -
            feedforward
        )

        return M_d

    def compute(
        self,
        p,
        v,
        R,
        omega,
        desired
    ):

        e_p, e_v = self.compute_position_errors(
            p,
            v,
            desired
        )

        F_d = self.compute_desired_force(
            e_p,
            e_v,
            desired["a_d"]
        )

        R_d = self.compute_desired_attitude(
            F_d,
            desired["yaw_d"]
        )

        e_R = self.compute_attitude_error(
            R,
            R_d
        )

        e_omega = self.compute_angular_velocity_error(
            R,
            R_d,
            omega,
            desired["omega_d"]
        )

        M_d = self.compute_moment(
            R,
            R_d,
            omega,
            desired["omega_d"],
            desired["omega_dot_d"],
            e_R,
            e_omega
        )

        return {
            "e_p": e_p,
            "e_v": e_v,
            "F_d": F_d,
            "R_d": R_d,
            "e_R": e_R,
            "e_omega": e_omega,
            "M_d": M_d
        }
