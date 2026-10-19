"""
===============================================================================
Model 2: Data-Driven Linear Model (DMDc) and Optimal Control (LQR / MPC) Node
===============================================================================
Executes the full closed loop from Model 2:
Nonlinear Flight Data -> DMDc Identification -> LQR / MPC Optimal Control -> Actuation
===============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import String, Float64
import numpy as np

from .ardupilot_interface import ArduPilotInterface
from .dmdc_identifier import DMDcIdentifier
from .optimal_control import OptimalControlEngine
from .target_manager import TargetManager
from .safety_manager import SafetyManager
from .command_interface import CommandInterface, ControlCommand
from .flight_manager import FlightManager
from .telemetry_logger import TelemetryLogger
from .state_adapter import so3_exp, rotation_matrix_to_quaternion


class Model2DMDcControllerNode(Node):

    def __init__(self):
        super().__init__('model2_dmdc_controller')

        # Declare parameters
        self.declare_parameter("rate", 25.0)  # Optimal control nominal frequency (25-50 Hz)
        self.declare_parameter("pose_topic", "/ap/v1/pose/filtered")
        self.declare_parameter("twist_topic", "/ap/v1/twist/filtered")
        self.declare_parameter("cmd_vel_topic", "/ap/v1/cmd_vel")
        self.declare_parameter("status_topic", "/ap/v1/status")
        self.declare_parameter("target_topic", "/drones_controller/target_pose")

        self.declare_parameter("control_mode", "mpc")  # "mpc" or "lqr"
        self.declare_parameter("target.x", 0.0)
        self.declare_parameter("target.y", 0.0)
        self.declare_parameter("target.z", 5.0)
        self.declare_parameter("target.yaw", 0.0)

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

        # DMDc and MPC settings
        self.declare_parameter("dmdc.window_size", 100)
        self.declare_parameter("dmdc.min_samples", 25)
        self.declare_parameter("mpc.horizon", 25)
        self.declare_parameter("mpc.q_pos", [10.0, 10.0, 20.0])
        self.declare_parameter("mpc.q_vel", [2.0, 2.0, 5.0])
        self.declare_parameter("mpc.q_rot", [5.0, 5.0, 2.0])
        self.declare_parameter("mpc.q_omega", [0.5, 0.5, 0.2])
        self.declare_parameter("mpc.r_weights", [0.1, 1.0, 1.0, 1.0])

        # Retrieve parameters
        self.control_rate = float(self.get_parameter("rate").value)
        self.dt = 1.0 / self.control_rate
        self.control_mode = str(self.get_parameter("control_mode").value).lower()
        self.min_takeoff_altitude = float(self.get_parameter("limits.min_takeoff_altitude").value)

        mass = float(self.get_parameter("mass").value)
        gravity = float(self.get_parameter("gravity").value)
        self.mg0 = mass * gravity
        inertia = list(self.get_parameter("inertia").value)
        max_vel = float(self.get_parameter("limits.max_velocity").value)
        pos_tol = float(self.get_parameter("limits.position_tolerance").value)
        vel_tol = float(self.get_parameter("limits.velocity_tolerance").value)
        state_timeout = float(self.get_parameter("safety.state_timeout").value)

        # Assemble Q & R weight vectors
        q_pos = list(self.get_parameter("mpc.q_pos").value)
        q_vel = list(self.get_parameter("mpc.q_vel").value)
        q_rot = list(self.get_parameter("mpc.q_rot").value)
        q_omega = list(self.get_parameter("mpc.q_omega").value)
        q_weights = q_pos + q_vel + q_rot + q_omega
        r_weights = list(self.get_parameter("mpc.r_weights").value)
        horizon = int(self.get_parameter("mpc.horizon").value)

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

        # Blocks 1 & 2: DMDc System Identifier
        self.dmdc = DMDcIdentifier(
            mass=mass,
            gravity=gravity,
            inertia=inertia,
            dt=self.dt,
            max_samples=int(self.get_parameter("dmdc.window_size").value),
            min_fit_samples=int(self.get_parameter("dmdc.min_samples").value)
        )

        # Blocks 3–9: Optimal Control Engine (DARE, LQR, Constrained QP MPC)
        A_init, B_init = self.dmdc.get_matrices()
        self.optimal_engine = OptimalControlEngine(
            A=A_init,
            B=B_init,
            q_weights=q_weights,
            r_weights=r_weights,
            horizon=horizon,
            dt=self.dt,
            max_vel=max_vel,
            t_min=5.0,
            t_max=25.0,
            mg0=self.mg0
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

        # Telemetry & Diagnostic Publishers (identical interface as Model 1 for seamless comparison)
        self.position_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/position_error", 10)
        self.velocity_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/velocity_error", 10)
        self.desired_force_pub = self.create_publisher(Vector3Stamped, "/drones_controller/desired_force", 10)
        self.desired_moment_pub = self.create_publisher(Vector3Stamped, "/drones_controller/desired_moment", 10)
        self.attitude_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/attitude_error", 10)
        self.angular_velocity_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/angular_velocity_error", 10)
        self.desired_attitude_pub = self.create_publisher(PoseStamped, "/drones_controller/desired_attitude", 10)
        self.optimal_cost_pub = self.create_publisher(Float64, "/drones_controller/optimal_cost", 10)
        self.dmdc_residual_pub = self.create_publisher(Float64, "/drones_controller/dmdc_residual", 10)
        self.controller_type_pub = self.create_publisher(String, "/drones_controller/controller_type", 10)

        # Target Setpoint Subscription
        target_topic = self.get_parameter("target_topic").value
        self.target_sub = self.create_subscription(PoseStamped, target_topic, self._target_pose_callback, 10)

        # Teleoperation Subscription
        self.teleop_active = False
        self.last_teleop_time = 0.0
        self.teleop_vx = 0.0
        self.teleop_vy = 0.0
        self.teleop_vz = 0.0
        self.teleop_yaw_rate = 0.0
        self.teleop_sub = self.create_subscription(TwistStamped, "/drones_controller/teleop_cmd", self._teleop_cmd_callback, 10)

        # Airborne tracking notification flag
        self._airborne_notified = False
        self._iteration_count = 0
        self._last_u = np.array([self.mg0, 0.0, 0.0, 0.0])

        # Control loop timer
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"Model 2: Data-Driven DMDc & Optimal Control ({self.control_mode.upper()}) started at {self.control_rate} Hz")

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
                self.get_logger().info(f"🕹️ [Model 2 {self.control_mode.upper()}] Teleop ENGAGED: vx={vx:+.2f}, vy={vy:+.2f}, vz={vz:+.2f}, yaw={wz:+.2f}")
            self.teleop_active = True
            self.last_teleop_time = now
        else:
            if self.teleop_active:
                self.teleop_active = False
                state = self.ardupilot_interface.get_state()
                if state is not None:
                    curr_yaw = self._get_current_yaw(state.quaternion)
                    self.target_manager.set_target_position(state.position[0], state.position[1], state.position[2], curr_yaw)
                    self.get_logger().info(f"[Model 2 {self.control_mode.upper()}] Position hold locked at: [{state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f}] m")

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
        self.get_logger().info(f"[Model 2 {self.control_mode.upper()}] Setpoint updated: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}")

    def publish_vector(self, publisher, vector: np.ndarray):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "model2_dmdc"
        msg.vector.x = float(vector[0])
        msg.vector.y = float(vector[1])
        msg.vector.z = float(vector[2])
        publisher.publish(msg)

    def control_loop(self):
        state = self.ardupilot_interface.get_state()
        if not self.safety_manager.validate_state(state):
            self.flight_manager.update(is_connected=False, target_reached=False)
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._iteration_count += 1

        # Teleop timeout check
        if self.teleop_active and (now_sec - self.last_teleop_time > 0.4):
            self.teleop_active = False
            curr_yaw = self._get_current_yaw(state.quaternion)
            self.target_manager.set_target_position(state.position[0], state.position[1], state.position[2], curr_yaw)
            self.get_logger().info(f"[Model 2 {self.control_mode.upper()}] Teleop timeout -> Position hold locked at: [{state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f}] m")

        target = self.target_manager.get_target()

        # 1. Block 1: Extract state deviation x_k = [p - p_d; v; Log(R); ω] ∈ R^12
        x_k = self.dmdc.extract_state_deviation(
            p=state.position,
            v=state.velocity,
            R=state.rotation,
            omega=state.angular_velocity,
            p_ref=target.position
        )

        # Extract applied input deviation u_{k-1} from last step
        u_prev_dev = self.dmdc.extract_input_deviation(self._last_u[0], self._last_u[1:4])
        self.dmdc.add_sample(x_k, u_prev_dev)

        # 2. Block 2: Periodic DMDc identification update (every 20 steps = ~0.8s)
        if self._iteration_count % 20 == 0:
            if self.dmdc.fit():
                A_new, B_new = self.dmdc.get_matrices()
                self.optimal_engine.update_model(A_new, B_new)
                res_msg = Float64()
                res_msg.data = self.dmdc.identification_residual
                self.dmdc_residual_pub.publish(res_msg)

        # 3. Blocks 3–9: Compute optimal input deviation u_opt ∈ R^4
        if self.control_mode == "lqr":
            u_opt = self.optimal_engine.compute_lqr(x_k)
        else:
            u_opt = self.optimal_engine.compute_mpc(x_k)

        # Optimal value function cost V(x) = x^T * P * x
        opt_cost = self.optimal_engine.compute_value_function(x_k)
        cost_msg = Float64()
        cost_msg.data = opt_cost
        self.optimal_cost_pub.publish(cost_msg)

        # 4. Block 10: Actuation - Total control input u_k = u_trim + u_opt
        total_T = float(self.mg0 + u_opt[0])
        tau = u_opt[1:4]
        self._last_u = np.array([total_T, tau[0], tau[1], tau[2]])

        # 5. Coordinate command synthesis for SITL (/ap/v1/cmd_vel)
        # Position error e_p = p_d - p = -x_k[0:3]
        e_p = target.position - state.position
        e_v = target.velocity - state.velocity
        e_R = -x_k[6:9]   # Attitude error vector
        e_omega = -x_k[9:12]

        max_vel = float(self.get_parameter("limits.max_velocity").value)
        # Translational velocity command derived from optimal position error damping
        v_cmd_raw = 0.5 * e_p + 0.2 * e_v
        speed = np.linalg.norm(v_cmd_raw)
        if speed > max_vel and speed > 1e-6:
            v_cmd = v_cmd_raw * (max_vel / speed)
        else:
            v_cmd = v_cmd_raw

        # Yaw rate command
        target_yaw = float(target.yaw)
        curr_yaw = self._get_current_yaw(state.quaternion)
        yaw_err = float(np.arctan2(np.sin(target_yaw - curr_yaw), np.cos(target_yaw - curr_yaw)))
        yaw_rate_cmd = float(np.clip(1.0 * yaw_err, -1.0, 1.0))

        # 6. Arbitrate between Human Pilot Teleop & Autonomous Optimal Control
        if self.teleop_active:
            if now_sec - getattr(self, '_last_teleop_log_time', 0.0) >= 1.0:
                self._last_teleop_log_time = now_sec
                self.get_logger().info(f"🕹️ [Model 2 {self.control_mode.upper()}] Teleop ACTIVE: vx={self.teleop_vx:+.2f}, vy={self.teleop_vy:+.2f}, vz={self.teleop_vz:+.2f}, yaw={self.teleop_yaw_rate:+.2f}")

            raw_command = ControlCommand(
                v_cmd=np.array([self.teleop_vx, self.teleop_vy, self.teleop_vz], dtype=float),
                yaw_rate=self.teleop_yaw_rate,
                frame_id="base_link",
                e_p=e_p,
                e_v=e_v,
                F_d=np.array([0.0, 0.0, total_T]),
                R_d=state.rotation,
                M_d=tau,
                thrust_d=total_T
            )
            curr_yaw = self._get_current_yaw(state.quaternion)
            self.target_manager.set_target_position(state.position[0], state.position[1], state.position[2], curr_yaw)
        else:
            raw_command = ControlCommand(
                v_cmd=v_cmd,
                yaw_rate=yaw_rate_cmd,
                frame_id="map",
                e_p=e_p,
                e_v=e_v,
                F_d=np.array([0.0, 0.0, total_T]),
                R_d=state.rotation,
                M_d=tau,
                thrust_d=total_T
            )

        safe_command = self.safety_manager.validate_command(raw_command)

        # 7. Airborne handoff gate
        pos_tol = float(self.get_parameter("limits.position_tolerance").value)
        vel_tol = float(self.get_parameter("limits.velocity_tolerance").value)
        target_reached = bool(np.linalg.norm(e_p) < pos_tol and np.linalg.norm(state.velocity) < vel_tol)

        self.flight_manager.update(
            is_connected=self.ardupilot_interface.is_connected(),
            target_reached=target_reached,
            armed=self.ardupilot_interface.armed,
            flying=self.ardupilot_interface.flying,
            failsafe=self.ardupilot_interface.failsafe
        )

        enable_cmd = bool(self.get_parameter("safety.enable_command").value) or bool(self.get_parameter("enable_command").value)
        is_airborne = float(state.position[2]) >= self.min_takeoff_altitude

        if enable_cmd and is_airborne:
            if not self._airborne_notified:
                self.get_logger().info(f"[Model 2 {self.control_mode.upper()}] Drone airborne (Z: {state.position[2]:.2f}m >= {self.min_takeoff_altitude:.2f}m) - Optimal Control ENGAGED!")
                self._airborne_notified = True
            self.command_interface.send(safe_command)
        else:
            if self._airborne_notified:
                self.get_logger().info(f"[Model 2 {self.control_mode.upper()}] Drone grounded (Z: {state.position[2]:.2f}m < {self.min_takeoff_altitude:.2f}m) - Pausing velocity commands.")
                self._airborne_notified = False

        # 8. Publish Telemetry for Web Dashboard & Comparative Analysis
        self.publish_vector(self.position_error_pub, e_p)
        self.publish_vector(self.velocity_error_pub, e_v)
        F_d = np.array([total_T * np.sin(-x_k[7]), total_T * np.sin(x_k[6]), total_T])
        self.publish_vector(self.desired_force_pub, F_d)
        self.publish_vector(self.desired_moment_pub, tau)
        self.publish_vector(self.attitude_error_pub, e_R)
        self.publish_vector(self.angular_velocity_error_pub, e_omega)

        q_d = rotation_matrix_to_quaternion(state.rotation @ so3_exp(e_R))
        attitude_msg = PoseStamped()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = "map"
        attitude_msg.pose.orientation.x = float(q_d[0])
        attitude_msg.pose.orientation.y = float(q_d[1])
        attitude_msg.pose.orientation.z = float(q_d[2])
        attitude_msg.pose.orientation.w = float(q_d[3])
        self.desired_attitude_pub.publish(attitude_msg)

        type_msg = String()
        type_msg.data = f"MODEL_2_DMDc_{self.control_mode.upper()}"
        self.controller_type_pub.publish(type_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Model2DMDcControllerNode()
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
