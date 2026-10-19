"""
===============================================================================
Model 1: Nonlinear Model-Based Estimation and Geometric SO(3) Control Node
===============================================================================
Executes the full closed loop from Model 1:
Plant -> Sensors -> Error-State EKF -> Outer Loop -> Guidance -> Inner Loop -> Actuators
===============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import String
import numpy as np

from .ardupilot_interface import ArduPilotInterface
from .error_state_ekf import ErrorStateEKF
from .position_controller import PositionController
from .attitude_generator import AttitudeGenerator
from .so3_controller import SO3Controller
from .target_manager import TargetManager
from .safety_manager import SafetyManager
from .command_interface import CommandInterface, ControlCommand
from .flight_manager import FlightManager
from .telemetry_logger import TelemetryLogger
from .state_adapter import rotation_matrix_to_quaternion


class Model1SO3ControllerNode(Node):

    def __init__(self):
        super().__init__('model1_so3_controller')

        # Declare parameters
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("pose_topic", "/ap/v1/pose/filtered")
        self.declare_parameter("twist_topic", "/ap/v1/twist/filtered")
        self.declare_parameter("cmd_vel_topic", "/ap/v1/cmd_vel")
        self.declare_parameter("status_topic", "/ap/v1/status")
        self.declare_parameter("target_topic", "/drones_controller/target_pose")

        self.declare_parameter("target.x", 0.0)
        self.declare_parameter("target.y", 0.0)
        self.declare_parameter("target.z", 5.0)
        self.declare_parameter("target.yaw", 0.0)

        self.declare_parameter("position.kp", [0.5, 0.5, 0.5])
        self.declare_parameter("position.kv", [0.2, 0.2, 0.2])

        self.declare_parameter("limits.max_velocity", 2.5)
        self.declare_parameter("limits.position_tolerance", 0.10)
        self.declare_parameter("limits.velocity_tolerance", 0.05)
        self.declare_parameter("limits.min_takeoff_altitude", 0.50)

        self.declare_parameter("safety.state_timeout", 0.2)
        self.declare_parameter("safety.enable_command", True)
        self.declare_parameter("enable_command", True)

        self.declare_parameter("mass", 1.5)
        self.declare_parameter("gravity", 9.81)
        self.declare_parameter("inertia", [0.02, 0.02, 0.04])
        self.declare_parameter("kR", [4.0, 4.0, 2.0])
        self.declare_parameter("kOmega", [0.8, 0.8, 0.5])

        self.declare_parameter("ekf.enable", True)

        # Retrieve parameters
        self.control_rate = float(self.get_parameter("rate").value)
        self.dt = 1.0 / self.control_rate
        self.min_takeoff_altitude = float(self.get_parameter("limits.min_takeoff_altitude").value)
        self.use_ekf = bool(self.get_parameter("ekf.enable").value)

        mass = float(self.get_parameter("mass").value)
        gravity = float(self.get_parameter("gravity").value)
        inertia = list(self.get_parameter("inertia").value)
        kp = list(self.get_parameter("position.kp").value)
        kv = list(self.get_parameter("position.kv").value)
        kR = list(self.get_parameter("kR").value)
        kOmega = list(self.get_parameter("kOmega").value)
        max_vel = float(self.get_parameter("limits.max_velocity").value)
        pos_tol = float(self.get_parameter("limits.position_tolerance").value)
        vel_tol = float(self.get_parameter("limits.velocity_tolerance").value)
        state_timeout = float(self.get_parameter("safety.state_timeout").value)

        # Initialize sub-modules
        self.ardupilot_interface = ArduPilotInterface(
            node=self,
            pose_topic=self.get_parameter("pose_topic").value,
            twist_topic=self.get_parameter("twist_topic").value,
            cmd_vel_topic=self.get_parameter("cmd_vel_topic").value,
            status_topic=self.get_parameter("status_topic").value,
            state_timeout=state_timeout
        )

        self.target_manager = TargetManager(
            initial_position=[
                self.get_parameter("target.x").value,
                self.get_parameter("target.y").value,
                self.get_parameter("target.z").value
            ],
            initial_yaw=self.get_parameter("target.yaw").value
        )

        # Block 4: Error-State EKF
        self.ekf = ErrorStateEKF(
            mass=mass,
            gravity=gravity,
            inertia=inertia
        )

        # Block 5: Position Controller (Outer Loop)
        self.position_controller = PositionController(
            mass=mass,
            gravity=gravity,
            kp=kp,
            kv=kv,
            max_velocity=max_vel,
            position_tolerance=pos_tol,
            velocity_tolerance=vel_tol
        )

        # Block 6: Guidance Force & Attitude Generator
        self.attitude_generator = AttitudeGenerator()

        # Block 7: SO(3) Attitude Controller (Inner Loop)
        self.so3_controller = SO3Controller(
            inertia=inertia,
            kR=kR,
            kOmega=kOmega
        )

        self.safety_manager = SafetyManager(
            max_velocity=max_vel,
            state_timeout=state_timeout
        )

        self.command_interface = CommandInterface(
            ardupilot_interface=self.ardupilot_interface
        )

        self.flight_manager = FlightManager(
            position_tolerance=pos_tol,
            velocity_tolerance=vel_tol
        )

        self.telemetry_logger = TelemetryLogger()

        # Telemetry Publishers
        self.position_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/position_error", 10)
        self.velocity_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/velocity_error", 10)
        self.desired_force_pub = self.create_publisher(Vector3Stamped, "/drones_controller/desired_force", 10)
        self.desired_moment_pub = self.create_publisher(Vector3Stamped, "/drones_controller/desired_moment", 10)
        self.attitude_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/attitude_error", 10)
        self.angular_velocity_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/angular_velocity_error", 10)
        self.desired_attitude_pub = self.create_publisher(PoseStamped, "/drones_controller/desired_attitude", 10)
        self.controller_type_pub = self.create_publisher(String, "/drones_controller/controller_type", 10)

        # Subscriptions
        target_topic = self.get_parameter("target_topic").value
        self.target_sub = self.create_subscription(PoseStamped, target_topic, self._target_pose_callback, 10)

        self.teleop_active = False
        self.last_teleop_time = 0.0
        self.teleop_vx = 0.0
        self.teleop_vy = 0.0
        self.teleop_vz = 0.0
        self.teleop_yaw_rate = 0.0
        self.teleop_sub = self.create_subscription(TwistStamped, "/drones_controller/teleop_cmd", self._teleop_cmd_callback, 10)

        # Airborne tracking notification flag
        self._airborne_notified = False
        self._last_u = np.array([mass * gravity, 0.0, 0.0, 0.0])

        # 50 Hz control loop
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"Model 1: Geometric SO(3) Controller started at {self.control_rate} Hz (Target topic: {target_topic})")

    def _get_current_yaw(self, q):
        siny_cosp = 2.0 * (q[3] * q[2] + q[0] * q[1])
        cosy_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
        return float(np.arctan2(siny_cosp, cosy_cosp))

    def _teleop_cmd_callback(self, msg: TwistStamped):
        now = self.get_clock().now().nanoseconds * 1e-9
        vx = float(msg.twist.linear.x)
        vy = float(msg.twist.linear.y)
        vz = float(msg.twist.linear.z)
        wz = float(msg.twist.angular.z)

        self.teleop_vx = vx
        self.teleop_vy = vy
        self.teleop_vz = vz
        self.teleop_yaw_rate = wz

        is_stick_moving = (abs(vx) > 0.02 or abs(vy) > 0.02 or abs(vz) > 0.02 or abs(wz) > 0.02)
        if is_stick_moving:
            if not self.teleop_active:
                self.get_logger().info(f"🕹️ [Model 1 SO(3)] Teleop ENGAGED: vx={vx:+.2f}, vy={vy:+.2f}, vz={vz:+.2f}, yaw={wz:+.2f}")
            self.teleop_active = True
            self.last_teleop_time = now
        else:
            if self.teleop_active:
                self.teleop_active = False
                state = self.ardupilot_interface.get_state()
                if state is not None:
                    curr_yaw = self._get_current_yaw(state.quaternion)
                    self.target_manager.set_target_position(state.position[0], state.position[1], state.position[2], curr_yaw)
                    self.get_logger().info(f"[Model 1 SO(3)] Position hold locked at: [{state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f}] m")

    def _target_pose_callback(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        q = msg.pose.orientation
        norm_q = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm_q > 0.01:
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = float(np.arctan2(siny_cosp, cosy_cosp))
        else:
            yaw = 0.0
        self.target_manager.set_target_position(x, y, z, yaw)
        self.get_logger().info(f"[Model 1 SO(3)] Setpoint updated: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}")

    def publish_vector(self, publisher, vector: np.ndarray):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "model1_so3"
        msg.vector.x = float(vector[0])
        msg.vector.y = float(vector[1])
        msg.vector.z = float(vector[2])
        publisher.publish(msg)

    def control_loop(self):
        # 1. State feedback from sensors / ArduPilot
        raw_state = self.ardupilot_interface.get_state()
        if not self.safety_manager.validate_state(raw_state):
            self.flight_manager.update(is_connected=False, target_reached=False)
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # 2. Block 4: Error-State EKF
        if self.use_ekf:
            self.ekf.predict(self._last_u[0], self._last_u[1:4], self.dt)
            self.ekf.update(raw_state.position, raw_state.rotation, raw_state.angular_velocity)
            ekf_out = self.ekf.get_state()
            p_est = ekf_out["p"]
            v_est = ekf_out["v"]
            R_est = ekf_out["R"]
            omega_est = ekf_out["omega"]
        else:
            p_est = raw_state.position
            v_est = raw_state.velocity
            R_est = raw_state.rotation
            omega_est = raw_state.angular_velocity

        # Teleop timeout check
        if self.teleop_active and (now_sec - self.last_teleop_time > 0.4):
            self.teleop_active = False
            curr_yaw = self._get_current_yaw(raw_state.quaternion)
            self.target_manager.set_target_position(p_est[0], p_est[1], p_est[2], curr_yaw)
            self.get_logger().info(f"[Model 1 SO(3)] Teleop timeout -> Position hold locked at: [{p_est[0]:.2f}, {p_est[1]:.2f}, {p_est[2]:.2f}] m")

        # 3. Read target setpoint
        target = self.target_manager.get_target()

        # 4. Block 5: Position Controller (Outer Loop)
        pos_output = self.position_controller.compute(
            p=p_est,
            v=v_est,
            p_d=target.position,
            v_d=target.velocity,
            a_d=target.acceleration
        )

        # 5. Block 6: Guidance Force & Attitude Generator
        att_target = self.attitude_generator.compute(
            F_d=pos_output.F_d,
            yaw_d=target.yaw
        )

        # 6. Block 7: SO(3) Attitude Controller (Inner Loop)
        so3_output = self.so3_controller.compute(
            R=R_est,
            omega=omega_est,
            R_d=att_target.R_d,
            omega_d=target.angular_velocity,
            omega_dot_d=target.angular_acceleration
        )

        # Save applied input for EKF prediction next cycle
        total_thrust = float(np.dot(pos_output.F_d, R_est[:, 2]))
        self._last_u = np.array([total_thrust, so3_output.M_d[0], so3_output.M_d[1], so3_output.M_d[2]])

        # 7. Arbitrate between Pilot Teleop & Autonomous Setpoint
        if self.teleop_active:
            if now_sec - getattr(self, '_last_teleop_log_time', 0.0) >= 1.0:
                self._last_teleop_log_time = now_sec
                self.get_logger().info(f"🕹️ [Model 1 SO(3)] Teleop ACTIVE: vx={self.teleop_vx:+.2f}, vy={self.teleop_vy:+.2f}, vz={self.teleop_vz:+.2f}, yaw={self.teleop_yaw_rate:+.2f}")

            raw_command = ControlCommand(
                v_cmd=np.array([self.teleop_vx, self.teleop_vy, self.teleop_vz], dtype=float),
                yaw_rate=self.teleop_yaw_rate,
                frame_id="base_link",
                e_p=pos_output.e_p,
                e_v=pos_output.e_v,
                F_d=pos_output.F_d,
                R_d=att_target.R_d,
                M_d=so3_output.M_d,
                thrust_d=att_target.thrust_d
            )
            curr_yaw = self._get_current_yaw(raw_state.quaternion)
            self.target_manager.set_target_position(p_est[0], p_est[1], p_est[2], curr_yaw)
        else:
            raw_command = ControlCommand(
                v_cmd=pos_output.v_cmd.copy(),
                yaw_rate=0.0,
                frame_id="map",
                e_p=pos_output.e_p,
                e_v=pos_output.e_v,
                F_d=pos_output.F_d,
                R_d=att_target.R_d,
                M_d=so3_output.M_d,
                thrust_d=att_target.thrust_d
            )

        safe_command = self.safety_manager.validate_command(raw_command)

        # 8. Flight state machine & Airborne handoff gate
        self.flight_manager.update(
            is_connected=self.ardupilot_interface.is_connected(),
            target_reached=pos_output.target_reached,
            armed=self.ardupilot_interface.armed,
            flying=self.ardupilot_interface.flying,
            failsafe=self.ardupilot_interface.failsafe
        )

        enable_cmd = bool(self.get_parameter("safety.enable_command").value) or bool(self.get_parameter("enable_command").value)
        is_airborne = float(p_est[2]) >= self.min_takeoff_altitude

        if enable_cmd and is_airborne:
            if not self._airborne_notified:
                self.get_logger().info(f"[Model 1 SO(3)] Drone is airborne (Altitude: {p_est[2]:.2f}m >= {self.min_takeoff_altitude:.2f}m) - Autonomous SO(3) velocity tracking ENGAGED!")
                self._airborne_notified = True
            self.command_interface.send(safe_command)
        else:
            if self._airborne_notified:
                self.get_logger().info(f"[Model 1 SO(3)] Drone grounded (Altitude: {p_est[2]:.2f}m < {self.min_takeoff_altitude:.2f}m) - Pausing velocity commands to allow clean MAVLink takeoff/landing.")
                self._airborne_notified = False

        # 9. Publish telemetry for Web Dashboard and Comparison
        self.publish_vector(self.position_error_pub, pos_output.e_p)
        self.publish_vector(self.velocity_error_pub, pos_output.e_v)
        self.publish_vector(self.desired_force_pub, pos_output.F_d)
        self.publish_vector(self.desired_moment_pub, so3_output.M_d)
        self.publish_vector(self.attitude_error_pub, so3_output.e_R)
        self.publish_vector(self.angular_velocity_error_pub, so3_output.e_omega)

        q_d = rotation_matrix_to_quaternion(att_target.R_d)
        attitude_msg = PoseStamped()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = "map"
        attitude_msg.pose.orientation.x = float(q_d[0])
        attitude_msg.pose.orientation.y = float(q_d[1])
        attitude_msg.pose.orientation.z = float(q_d[2])
        attitude_msg.pose.orientation.w = float(q_d[3])
        self.desired_attitude_pub.publish(attitude_msg)

        type_msg = String()
        type_msg.data = "MODEL_1_SO3_GEOMETRIC"
        self.controller_type_pub.publish(type_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Model1SO3ControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
