import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
import sys
import select
import termios
import tty
import threading

try:
    from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
    HAS_ARDUPILOT_MSGS = True
except ImportError:
    HAS_ARDUPILOT_MSGS = False


class DroneTeleop(Node):

    def __init__(self):
        super().__init__('drone_teleop')

        ap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(
            TwistStamped,
            '/ap/v1/cmd_vel',
            ap_qos
        )

        if HAS_ARDUPILOT_MSGS:
            self.arm_client = self.create_client(ArmMotors, '/ap/v1/arm_motors')
            self.mode_client = self.create_client(ModeSwitch, '/ap/v1/mode_switch')
            self.takeoff_client = self.create_client(Takeoff, '/ap/v1/experimental/takeoff')

        self.speed = 1.0
        self.yaw_speed = 0.5
        self.frame_id = 'base_link'

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wz = 0.0

        self.running = True

        self.get_logger().info('Drone Arrow-Key Teleop started with body-frame (base_link) control')

        self.print_banner()

        self.thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True
        )
        self.thread.start()

        self.timer = self.create_timer(
            0.1,  # 10 Hz continuous publish
            self.publish_command
        )

    def print_banner(self):
        print("""
======================================================
              🛸 DRONE RC KEYBOARD TELEOP
======================================================
  [Horizontal Flight - Body Frame (base_link)]:
    ▲ UP ARROW    : Forward (+X)
    ▼ DOWN ARROW  : Backward (-X)
    ◄ LEFT ARROW  : Move Left (+Y)
    ► RIGHT ARROW : Move Right (-Y)

  [Vertical Flight (Altitude)]:
    A             : Climb UP (+Z)
    Z             : Climb DOWN (-Z)

  [Yaw Rotation]:
    Q / E         : Yaw Rotate Left / Right

  [Brake & Speed]:
    SPACE         : STOP / HOVER (Hold Position)
    + / =         : Increase speed (+0.2 m/s)
    - / _         : Decrease speed (-0.2 m/s)

  [Vehicle Actions]:
    G             : Switch Mode to GUIDED (mode 4)
    M             : Arm Motors
    T             : Takeoff to 5m

  CTRL+C          : EXIT
======================================================
""")

    def call_mode_guided(self):
        if not HAS_ARDUPILOT_MSGS or not self.mode_client.service_is_ready():
            self.get_logger().warn("ModeSwitch service not ready")
            return
        req = ModeSwitch.Request()
        req.mode = 4
        self.mode_client.call_async(req)
        self.get_logger().info("Sent request: ModeSwitch -> GUIDED")

    def call_arm(self, arm_state: bool):
        if not HAS_ARDUPILOT_MSGS or not self.arm_client.service_is_ready():
            self.get_logger().warn("ArmMotors service not ready")
            return
        req = ArmMotors.Request()
        req.arm = arm_state
        self.arm_client.call_async(req)
        self.get_logger().info(f"Sent request: ArmMotors -> {'ARM' if arm_state else 'DISARM'}")

    def call_takeoff(self, alt: float = 5.0):
        if not HAS_ARDUPILOT_MSGS or not self.takeoff_client.service_is_ready():
            self.get_logger().warn("Takeoff service not ready")
            return
        req = Takeoff.Request()
        req.alt = float(alt)
        self.takeoff_client.call_async(req)
        self.get_logger().info(f"Sent request: Takeoff to {alt}m")

    def read_key(self):
        """Read standard keys and 3-byte ANSI arrow key escape sequences."""
        if not select.select([sys.stdin], [], [], 0.05)[0]:
            return None

        ch = sys.stdin.read(1)
        if ch == '\x1b':
            # Check for escape sequence
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    if select.select([sys.stdin], [], [], 0.05)[0]:
                        ch3 = sys.stdin.read(1)
                        if ch3 == 'A':
                            return 'KEY_UP'
                        elif ch3 == 'B':
                            return 'KEY_DOWN'
                        elif ch3 == 'C':
                            return 'KEY_RIGHT'
                        elif ch3 == 'D':
                            return 'KEY_LEFT'
            return '\x1b'
        return ch

    def keyboard_loop(self):
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin.fileno())

            while self.running:
                key = self.read_key()
                if key is None:
                    continue

                action = ""

                # Arrow keys for horizontal directional movement
                if key == 'KEY_UP' or key in ('w', 'W'):
                    self.vx = self.speed
                    self.vy = 0.0
                    action = "▲ FORWARD"
                elif key == 'KEY_DOWN' or key in ('s', 'S'):
                    self.vx = -self.speed
                    self.vy = 0.0
                    action = "▼ BACKWARD"
                elif key == 'KEY_LEFT':
                    self.vy = self.speed
                    self.vx = 0.0
                    action = "◄ MOVE LEFT"
                elif key == 'KEY_RIGHT':
                    self.vy = -self.speed
                    self.vx = 0.0
                    action = "► MOVE RIGHT"

                # 'a' for Climb UP, 'z' for Climb DOWN
                elif key in ('a', 'A'):
                    self.vz = self.speed
                    action = "▲ CLIMB UP"
                elif key in ('z', 'Z'):
                    self.vz = -self.speed
                    action = "▼ CLIMB DOWN"

                # Yaw rotation
                elif key in ('q', 'Q'):
                    self.wz = self.yaw_speed
                    action = "↺ YAW LEFT"
                elif key in ('e', 'E'):
                    self.wz = -self.yaw_speed
                    action = "↻ YAW RIGHT"

                # Stop / Hover
                elif key == ' ':
                    self.vx = 0.0
                    self.vy = 0.0
                    self.vz = 0.0
                    self.wz = 0.0
                    action = "■ HOVER / STOP"

                # Speed scaling
                elif key in ('+', '='):
                    self.speed = min(5.0, round(self.speed + 0.2, 1))
                    self.yaw_speed = min(2.0, round(self.yaw_speed + 0.1, 1))
                    action = f"SPEED+ ({self.speed:.1f} m/s)"
                elif key in ('-', '_'):
                    self.speed = max(0.2, round(self.speed - 0.2, 1))
                    self.yaw_speed = max(0.1, round(self.yaw_speed - 0.1, 1))
                    action = f"SPEED- ({self.speed:.1f} m/s)"

                # Vehicle state actions
                elif key in ('g', 'G'):
                    self.call_mode_guided()
                    action = "MODE -> GUIDED"
                elif key in ('m', 'M'):
                    self.call_arm(True)
                    action = "ARM MOTORS"
                elif key in ('t', 'T'):
                    self.call_takeoff(5.0)
                    action = "TAKEOFF 5M"

                print(
                    f"\r[ACT: {action:<16}] vx={self.vx:+.1f} vy={self.vy:+.1f} vz={self.vz:+.1f} yaw={self.wz:+.1f} | Speed: {self.speed:.1f} m/s   ",
                    end='',
                    flush=True
                )

        finally:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                old_settings
            )

    def publish_command(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.twist.linear.x = float(self.vx)
        msg.twist.linear.y = float(self.vy)
        msg.twist.linear.z = float(self.vz)

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(self.wz)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
