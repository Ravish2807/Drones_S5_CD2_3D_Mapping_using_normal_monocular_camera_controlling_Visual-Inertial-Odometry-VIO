import numpy as np


class WaypointGenerator:

    def __init__(self, position, yaw):

        self.position = np.array(position, dtype=float)
        self.yaw = float(yaw)

    def get_desired_state(self):

        p_d = self.position.copy()

        v_d = np.zeros(3)

        a_d = np.zeros(3)

        omega_d = np.zeros(3)

        omega_dot_d = np.zeros(3)

        return {
            "p_d": p_d,
            "v_d": v_d,
            "a_d": a_d,
            "yaw_d": self.yaw,
            "omega_d": omega_d,
            "omega_dot_d": omega_dot_d
        }
