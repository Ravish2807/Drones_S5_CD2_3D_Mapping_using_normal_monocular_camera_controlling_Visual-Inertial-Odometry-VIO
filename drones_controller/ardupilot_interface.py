import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

try:
    from ardupilot_msgs.msg import Status
    from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
    HAS_ARDUPILOT_MSGS = True
except ImportError:
    HAS_ARDUPILOT_MSGS = False

from .state_adapter import (
    DroneState,
    quaternion_to_rotation_matrix,
    validate_rotation_matrix
)


class ArduPilotInterface:

    def __init__(
        self,
        node: Node,
        pose_topic: str = "/ap/v1/pose/filtered",
        twist_topic: str = "/ap/v1/twist/filtered",
        cmd_vel_topic: str = "/ap/v1/cmd_vel",
        status_topic: str = "/ap/v1/status",
        state_timeout: float = 0.2
    ):
        self.node = node
        self.pose_topic = pose_topic
        self.twist_topic = twist_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.status_topic = status_topic
        self.state_timeout = float(state_timeout)

        self.position = None
        self.velocity = None
        self.rotation = None
        self.quaternion = None
        self.omega = None

        self.armed = False
        self.flying = False
        self.mode = 0
        self.external_control = False
        self.failsafe = False

        self.last_pose_time = None
        self.last_twist_time = None
        self.last_status_time = None

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pose_sub = node.create_subscription(
            PoseStamped,
            pose_topic,
            self._pose_callback,
            qos_profile
        )

        self.twist_sub = node.create_subscription(
            TwistStamped,
            twist_topic,
            self._twist_callback,
            qos_profile
        )

        if HAS_ARDUPILOT_MSGS:
            self.status_sub = node.create_subscription(
                Status,
                status_topic,
                self._status_callback,
                qos_profile
            )

        self.cmd_pub = node.create_publisher(
            TwistStamped,
            cmd_vel_topic,
            qos_profile
        )

        # Service clients
        if HAS_ARDUPILOT_MSGS:
            self.arm_client = node.create_client(ArmMotors, "/ap/v1/arm_motors")
            self.mode_client = node.create_client(ModeSwitch, "/ap/v1/mode_switch")
            self.takeoff_client = node.create_client(Takeoff, "/ap/v1/experimental/takeoff")

    def _pose_callback(self, msg: PoseStamped):
        self.position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ], dtype=float)

        self.quaternion = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ], dtype=float)

        try:
            R = quaternion_to_rotation_matrix(
                self.quaternion[0],
                self.quaternion[1],
                self.quaternion[2],
                self.quaternion[3]
            )
            if validate_rotation_matrix(R):
                self.rotation = R
            else:
                self.rotation = None
        except ValueError:
            self.rotation = None

        self.last_pose_time = self.node.get_clock().now()

    def _twist_callback(self, msg: TwistStamped):
        self.velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ], dtype=float)

        self.omega = np.array([
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z
        ], dtype=float)

        self.last_twist_time = self.node.get_clock().now()

    def _status_callback(self, msg):
        if hasattr(msg, 'armed'):
            self.armed = bool(msg.armed)
        if hasattr(msg, 'flying'):
            self.flying = bool(msg.flying)
        if hasattr(msg, 'mode'):
            self.mode = int(msg.mode)
        if hasattr(msg, 'external_control'):
            self.external_control = bool(msg.external_control)
        if hasattr(msg, 'failsafe'):
            self.failsafe = bool(msg.failsafe)
        self.last_status_time = self.node.get_clock().now()

    def is_connected(self) -> bool:
        if self.last_pose_time is None or self.last_twist_time is None:
            return False

        now = self.node.get_clock().now()
        pose_age = (now - self.last_pose_time).nanoseconds * 1e-9
        twist_age = (now - self.last_twist_time).nanoseconds * 1e-9

        return pose_age <= self.state_timeout and twist_age <= self.state_timeout

    def get_state(self) -> DroneState:
        if not self.is_connected() or self.position is None or self.rotation is None:
            return None

        now_sec = self.node.get_clock().now().nanoseconds * 1e-9

        return DroneState(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            rotation=self.rotation.copy(),
            quaternion=self.quaternion.copy(),
            angular_velocity=self.omega.copy(),
            timestamp=now_sec
        )

    def publish_cmd_vel(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0, frame_id: str = "map"):
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(yaw_rate)

        self.cmd_pub.publish(msg)
