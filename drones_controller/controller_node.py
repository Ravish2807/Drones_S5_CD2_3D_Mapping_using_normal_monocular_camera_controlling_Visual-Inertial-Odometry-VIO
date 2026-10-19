import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped, TwistStamped

from .ardupilot_interface import ArduPilotInterface
from .target_manager import TargetManager
from .trajectory_generator import WaypointGenerator
from .position_controller import PositionController
from .attitude_generator import AttitudeGenerator
from .so3_controller import SO3Controller
from .safety_manager import SafetyManager
from .command_interface import CommandInterface, ControlCommand
from .flight_manager import FlightManager
from .telemetry_logger import TelemetryLogger
from .state_adapter import rotation_matrix_to_quaternion


class ControllerNode(Node):

    def __init__(self):
        super().__init__("so3_controller")

        # Declare ROS 2 parameters
        self.declare_parameter("rate", 50.0)

        self.declare_parameter("pose_topic", "/ap/v1/pose/filtered")
        self.declare_parameter("twist_topic", "/ap/v1/twist/filtered")
        self.declare_parameter("cmd_vel_topic", "/ap/v1/cmd_vel")
        self.declare_parameter("status_topic", "/ap/v1/status")

        self.declare_parameter("target_topic", "/drones_controller/target_pose")

        self.declare_parameter("target.x", 1.0)
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

        # Parse parameters
        self.control_rate = float(self.get_parameter("rate").value)

        pose_topic = self.get_parameter("pose_topic").value
        twist_topic = self.get_parameter("twist_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        status_topic = self.get_parameter("status_topic").value

        target_x = float(self.get_parameter("target.x").value)
        target_y = float(self.get_parameter("target.y").value)
        target_z = float(self.get_parameter("target.z").value)
        target_yaw = float(self.get_parameter("target.yaw").value)

        kp = self.get_parameter("position.kp").value
        kv = self.get_parameter("position.kv").value

        max_velocity = float(self.get_parameter("limits.max_velocity").value)
        position_tolerance = float(self.get_parameter("limits.position_tolerance").value)
        velocity_tolerance = float(self.get_parameter("limits.velocity_tolerance").value)
        self.min_takeoff_altitude = float(self.get_parameter("limits.min_takeoff_altitude").value)
        self._airborne_notified = False

        state_timeout = float(self.get_parameter("safety.state_timeout").value)
        self.enable_command = bool(self.get_parameter("safety.enable_command").value) or bool(self.get_parameter("enable_command").value)

        mass = float(self.get_parameter("mass").value)
        gravity = float(self.get_parameter("gravity").value)
        inertia = self.get_parameter("inertia").value
        kR = self.get_parameter("kR").value
        kOmega = self.get_parameter("kOmega").value

        # Instantiate components
        self.ardupilot_interface = ArduPilotInterface(
            node=self,
            pose_topic=pose_topic,
            twist_topic=twist_topic,
            cmd_vel_topic=cmd_vel_topic,
            status_topic=status_topic,
            state_timeout=state_timeout
        )

        self.target_manager = TargetManager(
            initial_position=[target_x, target_y, target_z],
            initial_yaw=target_yaw
        )

        self.position_controller = PositionController(
            mass=mass,
            gravity=gravity,
            kp=kp,
            kv=kv,
            max_velocity=max_velocity,
            position_tolerance=position_tolerance,
            velocity_tolerance=velocity_tolerance
        )

        self.attitude_generator = AttitudeGenerator()

        self.so3_controller = SO3Controller(
            inertia=inertia,
            kR=kR,
            kOmega=kOmega
        )

        self.safety_manager = SafetyManager(
            max_velocity=max_velocity,
            state_timeout=state_timeout
        )

        self.command_interface = CommandInterface(
            ardupilot_interface=self.ardupilot_interface
        )

        self.flight_manager = FlightManager(
            position_tolerance=position_tolerance,
            velocity_tolerance=velocity_tolerance
        )
        self.telemetry_logger = TelemetryLogger()

        # Telemetry & SO(3) Debug Publishers
        self.position_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/position_error", 10)
        self.velocity_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/velocity_error", 10)
        self.desired_force_pub = self.create_publisher(Vector3Stamped, "/drones_controller/desired_force", 10)
        self.desired_moment_pub = self.create_publisher(Vector3Stamped, "/drones_controller/desired_moment", 10)
        self.attitude_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/attitude_error", 10)
        self.angular_velocity_error_pub = self.create_publisher(Vector3Stamped, "/drones_controller/angular_velocity_error", 10)
        self.desired_attitude_pub = self.create_publisher(PoseStamped, "/drones_controller/desired_attitude", 10)

        # Dynamic Target Setpoint Subscription (e.g. from MATLAB or remote devices)
        target_topic = self.get_parameter("target_topic").value
        self.target_sub = self.create_subscription(
            PoseStamped,
            target_topic,
            self._target_pose_callback,
            10
        )

        # Teleoperation Subscription (from Web Gamepad HUD, joystick_teleop node, or CLI)
        self.teleop_active = False
        self.last_teleop_time = 0.0
        self.teleop_vx = 0.0
        self.teleop_vy = 0.0
        self.teleop_vz = 0.0
        self.teleop_yaw_rate = 0.0

        self.teleop_sub = self.create_subscription(
            TwistStamped,
            "/drones_controller/teleop_cmd",
            self._teleop_cmd_callback,
            10
        )

        # Timer setup for continuous control loop
        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(f"Closed-loop Autonomous XYZ Position Controller started at {self.control_rate} Hz (Target topic: {target_topic})")

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
                self.get_logger().info(f"🕹️ Teleop ENGAGED: vx={vx:+.2f}, vy={vy:+.2f}, vz={vz:+.2f}, yaw={wz:+.2f}")
            self.teleop_active = True
            self.last_teleop_time = now
        else:
            if self.teleop_active:
                self.teleop_active = False
                state = self.ardupilot_interface.get_state()
                if state is not None:
                    curr_yaw = self._get_current_yaw(state.quaternion)
                    self.target_manager.set_target_position(
                        state.position[0], state.position[1], state.position[2], curr_yaw
                    )
                    self.get_logger().info(f"Joystick centered -> Position hold locked at: [{state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f}] m")

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
        self.get_logger().info(f"Target setpoint updated via ROS topic: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}")

    def publish_vector(self, publisher, vector: np.ndarray):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "controller"
        msg.vector.x = float(vector[0])
        msg.vector.y = float(vector[1])
        msg.vector.z = float(vector[2])
        publisher.publish(msg)

    def control_loop(self):
        # 1. Receive state feedback from ArduPilot Interface
        state = self.ardupilot_interface.get_state()

        # 2. Validate state safety
        if not self.safety_manager.validate_state(state):
            self.flight_manager.update(is_connected=False, target_reached=False)
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Check teleop timeout (if pilot stopped moving sticks or packet stream dropped)
        if self.teleop_active and (now_sec - self.last_teleop_time > 0.4):
            self.teleop_active = False
            curr_yaw = self._get_current_yaw(state.quaternion)
            self.target_manager.set_target_position(
                state.position[0], state.position[1], state.position[2], curr_yaw
            )
            self.get_logger().info(f"Teleop timeout -> Position hold locked at: [{state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f}] m")

        # 3. Read target setpoint
        target = self.target_manager.get_target()

        # 4. Position Controller -> velocity command v_cmd & target reached check
        pos_output = self.position_controller.compute(
            p=state.position,
            v=state.velocity,
            p_d=target.position,
            v_d=target.velocity,
            a_d=target.acceleration
        )

        # 5. SO(3) Attitude Generator & SO(3) Controller (for parallel research debug logging)
        att_target = self.attitude_generator.compute(
            F_d=pos_output.F_d,
            yaw_d=target.yaw
        )
        so3_output = self.so3_controller.compute(
            R=state.rotation,
            omega=state.angular_velocity,
            R_d=att_target.R_d,
            omega_d=target.angular_velocity,
            omega_dot_d=target.angular_acceleration
        )

        # 6. Apply command limits & safety validation (Teleop vs Autonomous Arbitrator)
        if self.teleop_active:
            # Human pilot teleoperation in drone body frame ('base_link')
            if now_sec - getattr(self, '_last_teleop_log_time', 0.0) >= 1.0:
                self._last_teleop_log_time = now_sec
                self.get_logger().info(f"🕹️ Teleop ACTIVE: vx={self.teleop_vx:+.2f}, vy={self.teleop_vy:+.2f}, vz={self.teleop_vz:+.2f}, yaw={self.teleop_yaw_rate:+.2f}")

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
            # Continually keep target_manager updated to current position so position hold is locked immediately upon stick release
            curr_yaw = self._get_current_yaw(state.quaternion)
            self.target_manager.set_target_position(
                state.position[0], state.position[1], state.position[2], curr_yaw
            )
        else:
            # Autonomous position tracking in world map frame ('map')
            cmd_v = pos_output.v_cmd.copy()
            if float(state.position[2]) < 1.5:
                # During initial climb / takeoff phase, zero lateral commands to prevent ground tipping/bouncing
                cmd_v[0] = 0.0
                cmd_v[1] = 0.0

            raw_command = ControlCommand(
                v_cmd=cmd_v,
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

        # 7. Update flight state machine
        flight_state = self.flight_manager.update(
            is_connected=self.ardupilot_interface.is_connected(),
            target_reached=pos_output.target_reached,
            armed=self.ardupilot_interface.armed,
            flying=self.ardupilot_interface.flying,
            failsafe=self.ardupilot_interface.failsafe
        )

        # 8. Command publishing: only when drone is airborne (Z >= min_takeoff_altitude)
        enable_cmd = bool(self.get_parameter("safety.enable_command").value) or bool(self.get_parameter("enable_command").value)
        is_airborne = float(state.position[2]) >= self.min_takeoff_altitude

        if enable_cmd and is_airborne:
            if not self._airborne_notified:
                self.get_logger().info(f"Drone is airborne (Altitude: {state.position[2]:.2f}m >= {self.min_takeoff_altitude:.2f}m) - Autonomous SO(3) velocity tracking ENGAGED!")
                self._airborne_notified = True
            self.command_interface.send(safe_command)
        else:
            if self._airborne_notified:
                self.get_logger().info(f"Drone grounded (Altitude: {state.position[2]:.2f}m < {self.min_takeoff_altitude:.2f}m) - Pausing velocity commands to allow clean MAVLink takeoff/landing.")
                self._airborne_notified = False

        # 9. Publish debug topics for SO(3) research
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

        # Log telemetry frame
        self.telemetry_logger.log(
            timestamp=state.timestamp,
            target_pos=target.position,
            actual_pos=state.position,
            pos_error=pos_output.e_p,
            actual_vel=state.velocity,
            vel_error=pos_output.e_v,
            desired_force=pos_output.F_d,
            desired_moment=so3_output.M_d,
            attitude_error=so3_output.e_R,
            angular_vel_error=so3_output.e_omega,
            desired_thrust=att_target.thrust_d,
            flight_state=flight_state.name
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
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


if __name__ == "__main__":
    main()
