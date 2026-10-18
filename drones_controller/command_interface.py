from dataclasses import dataclass
import numpy as np
from geometry_msgs.msg import TwistStamped
from .ardupilot_interface import ArduPilotInterface


@dataclass
class ControlCommand:
    v_cmd: np.ndarray           # Velocity command [vx, vy, vz]
    yaw_rate: float = 0.0       # Desired yaw rate
    frame_id: str = "map"       # Coordinate frame ("map" for ENU autonomous, "base_link" for body teleop)
    e_p: np.ndarray = None      # Position error for SO(3) debug
    e_v: np.ndarray = None      # Velocity error for SO(3) debug
    F_d: np.ndarray = None      # Desired force for SO(3) debug
    R_d: np.ndarray = None      # Desired rotation matrix for SO(3) debug
    M_d: np.ndarray = None      # Desired control moment for SO(3) debug
    thrust_d: float = 0.0       # Desired thrust magnitude for SO(3) debug


class CommandInterface:

    def __init__(self, ardupilot_interface: ArduPilotInterface):
        self.ardupilot = ardupilot_interface

    def send(self, command: ControlCommand):
        if command is None or command.v_cmd is None:
            self.ardupilot.publish_cmd_vel(0.0, 0.0, 0.0, 0.0, frame_id="map")
            return

        v = command.v_cmd
        vx = float(v[0])
        vy = float(v[1])
        vz = float(v[2])
        yaw_rate = float(command.yaw_rate)
        frame_id = getattr(command, 'frame_id', 'map')

        self.ardupilot.publish_cmd_vel(vx, vy, vz, yaw_rate, frame_id=frame_id)
