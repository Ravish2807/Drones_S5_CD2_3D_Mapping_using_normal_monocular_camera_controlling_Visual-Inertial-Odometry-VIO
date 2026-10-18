import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
import sys
import os
import select
import termios
import tty
import threading
import time
import struct
import array
import fcntl

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

        # Primary: publish to controller_node teleop topic for SO(3) arbitration
        self.teleop_pub = self.create_publisher(
            TwistStamped,
            '/drones_controller/teleop_cmd',
            10
        )

        # Fallback: direct to ArduPilot SITL if controller_node is offline
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/ap/v1/cmd_vel',
            ap_qos
        )

        if HAS_ARDUPILOT_MSGS:
            self.arm_client = self.create_client(ArmMotors, '/ap/v1/arm_motors')
            self.mode_client = self.create_client(ModeSwitch, '/ap/v1/mode_switch')
            self.takeoff_client = self.create_client(Takeoff, '/ap/v1/experimental/takeoff')

        self.speed = 1.0
        self.yaw_speed = 0.8
        self.vertical_speed = 0.8
        self.frame_id = 'base_link'

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wz = 0.0

        self.running = True
        self.js_name = "Searching..."
        self.js_device = None
        self.js_active = False

        self.get_logger().info('Drone Teleop started: Default ROS keyboard + 8BitDo / Gamepad integration')

        self.print_banner()

        # Keyboard reading thread
        self.kb_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True
        )
        self.kb_thread.start()

        # Joystick background reading thread
        self.js_thread = threading.Thread(
            target=self.joystick_loop,
            daemon=True
        )
        self.js_thread.start()

        # Continuous publish timer at 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.publish_command
        )

    def print_banner(self):
        print("""
======================================================================
         🛸 DRONE TELEOP: DEFAULT ROS KEYBOARD + 8BITDO GAMEPAD
======================================================================
  [Standard ROS teleop_twist_keyboard Navigation]:
    u    i    o         i : Forward (+X)       , : Backward (-X)
    j    k    l         j : Yaw Left (+Yaw)    l : Yaw Right (-Yaw)
    m    ,    .         u / o : Fwd + Turn     m / . : Back + Turn
                        k or SPACE : STOP / HOVER (Lock Position)

  [Holonomic Strafe (Shift + Keys)]:
    U    I    O         I : Forward            < : Backward
    J    K    L         J : Strafe Left (+Y)   L : Strafe Right (-Y)
    M    <    >         U / O : Fwd + Strafe   M / > : Back + Strafe

  [Altitude / Vertical]:
    t : Climb UP (+Z)              b : Climb DOWN (-Z)

  [Laptop Arrow & WASD Fallback]:
    ▲ UP / W    : Forward (+X)     ▼ DOWN / S  : Backward (-X)
    ◄ LEFT / A  : Strafe Left (+Y) ► RIGHT / D : Strafe Right (-Y)

  [Vehicle Actions]:
    y : Mode GUIDED                x : TAKEOFF 5m
    1 / r : ARM Motors             0 : DISARM Motors
    k / h / SPACE : HOVER / STOP

  [🎮 8BitDo Ultimate 2C / Gamepad RC Mode 2]:
    Left Stick  : Throttle (+Z/-Z Climb/Descend) & Yaw (Rotate L/R)
    Right Stick : Pitch (+X/-X Forward/Back) & Roll (+Y/-Y Left/Right)
    Buttons     : Y=GUIDED | A=ARM | X=TAKEOFF (5m) | B=HOVER/HOLD
    Bumpers     : LB=Precision (0.5 m/s) | RB=Turbo (2.0 m/s)

  [Speed Scaling]:
    q / z : Increase / decrease max speeds by 10%
    + / - : Increase / decrease linear speed (±0.2 m/s)

  CTRL+C : EXIT
======================================================================
""")

    def _call_bridge_api(self, endpoint, payload):
        try:
            import urllib.request
            import json
            req = urllib.request.Request(
                f"http://127.0.0.1:8000/api/{endpoint}",
                data=json.dumps(payload).encode('utf-8'),
                headers={'Content-Type': 'application/json'}
            )
            with urllib.request.urlopen(req, timeout=1.0) as resp:
                data = json.loads(resp.read().decode('utf-8'))
                return data.get('success', True), data.get('message', '')
        except Exception:
            return False, "Bridge server (port 8000) offline"

    def call_mode_guided(self):
        ok, msg = self._call_bridge_api("mode", {"mode": "GUIDED"})
        if ok:
            print("\r[MAVLink Bridge] Mode -> GUIDED successfully (Next: Press A to ARM)          ", flush=True)
            return
        if HAS_ARDUPILOT_MSGS and hasattr(self, 'mode_client') and self.mode_client.service_is_ready():
            req = ModeSwitch.Request()
            req.mode = 4
            self.mode_client.call_async(req)
            self.get_logger().info("Sent request: ModeSwitch -> GUIDED")
        else:
            self.get_logger().warn("Mode switch failed: ensure bridge_server (port 8000) is running")

    def call_arm(self, arm_state: bool):
        ok, msg = self._call_bridge_api("arm", {"arm": arm_state})
        if ok:
            status = 'ARMED' if arm_state else 'DISARMED'
            print(f"\r[MAVLink Bridge] Motors {status}! (Now press Button X to TAKEOFF to 5m)     ", flush=True)
            return
        if HAS_ARDUPILOT_MSGS and hasattr(self, 'arm_client') and self.arm_client.service_is_ready():
            req = ArmMotors.Request()
            req.arm = arm_state
            self.arm_client.call_async(req)
            self.get_logger().info(f"Sent request: ArmMotors -> {'ARM' if arm_state else 'DISARM'}")
        else:
            self.get_logger().warn("Arm motors failed: ensure bridge_server (port 8000) is running")

    def call_takeoff(self, alt: float = 5.0):
        ok, msg = self._call_bridge_api("takeoff", {"alt": alt})
        if ok:
            print(f"\r[MAVLink Bridge] TAKEOFF to {alt}m initiated! (Climbing straight up...)      ", flush=True)
            return
        if HAS_ARDUPILOT_MSGS and hasattr(self, 'takeoff_client') and self.takeoff_client.service_is_ready():
            req = Takeoff.Request()
            req.alt = float(alt)
            self.takeoff_client.call_async(req)
            self.get_logger().info(f"Sent request: Takeoff to {alt}m")
        else:
            self.get_logger().warn("Takeoff failed: ensure bridge_server (port 8000) is running")

    # =========================================================================
    # 8BitDo / Hardware Gamepad Thread
    # =========================================================================
    def _find_joystick(self):
        # Prioritize js1 (8BitDo Wireless), then js0, js2
        candidates = ["/dev/input/js1", "/dev/input/js0", "/dev/input/js2"]
        JSIOCGNAME = 0x80006a13 + (128 << 16)

        for p in candidates:
            if not os.path.exists(p):
                continue
            try:
                fd = os.open(p, os.O_RDONLY | os.O_NONBLOCK)
                buf = array.array('B', [0] * 128)
                fcntl.ioctl(fd, JSIOCGNAME, buf)
                name = buf.tobytes().decode('utf-8', errors='ignore').rstrip('\x00')
                os.close(fd)
                if "8BitDo" in name or "Wireless" in name:
                    return p, name
            except Exception:
                pass

        for p in candidates:
            if os.path.exists(p):
                try:
                    fd = os.open(p, os.O_RDONLY | os.O_NONBLOCK)
                    buf = array.array('B', [0] * 128)
                    fcntl.ioctl(fd, JSIOCGNAME, buf)
                    name = buf.tobytes().decode('utf-8', errors='ignore').rstrip('\x00')
                    os.close(fd)
                    if "input-remapper" not in name.lower():
                        return p, name
                except Exception:
                    pass

        return None, None

    def joystick_loop(self):
        js_fd = None
        raw_axes = {}
        raw_buttons = {}
        DEADZONE = 0.12

        yaw_axis = 0
        thr_axis = 1
        roll_axis = 2
        pitch_axis = 3

        while self.running:
            if js_fd is None:
                dev_path, dev_name = self._find_joystick()
                if dev_path:
                    try:
                        js_fd = os.open(dev_path, os.O_RDONLY | os.O_NONBLOCK)
                        self.js_device = dev_path
                        self.js_name = dev_name

                        # Query exact axis map directly from Linux joydev driver
                        axmap = bytearray(64)
                        axes_cnt = bytearray(1)
                        fcntl.ioctl(js_fd, 0x80016a11, axes_cnt) # JSIOCGAXES
                        n_axes = axes_cnt[0] if axes_cnt[0] > 0 else 8
                        fcntl.ioctl(js_fd, 0x80406a32, axmap)    # JSIOCGAXMAP
                        codes = list(axmap[:n_axes])

                        if 2 in codes and 5 in codes:
                            # 8BitDo Ultimate 2C Bluetooth / Linux evdev (ABS_Z=2 is Roll, ABS_RZ=5 is Pitch)
                            roll_axis = codes.index(2)
                            pitch_axis = codes.index(5)
                        elif 3 in codes and 4 in codes:
                            # Xbox 360 / xpad (ABS_RX=3 is Roll, ABS_RY=4 is Pitch)
                            roll_axis = codes.index(3)
                            pitch_axis = codes.index(4)
                        else:
                            roll_axis = 2
                            pitch_axis = 3

                        yaw_axis = codes.index(0) if 0 in codes else 0
                        thr_axis = codes.index(1) if 1 in codes else 1

                        print(f"\n[🎮 Gamepad Connected] {dev_name} at {dev_path}")
                        print(f"    Kernel axis codes: {codes}")
                        print(f"    Hardware Mapped -> Left:[Yaw={yaw_axis}, Thr={thr_axis}] | Right:[Roll={roll_axis}, Pitch={pitch_axis}]\n", flush=True)
                    except Exception as e:
                        roll_axis = 2
                        pitch_axis = 3
                        yaw_axis = 0
                        thr_axis = 1
                time.sleep(1.0)
                continue

            try:
                # Read 8 bytes: struct js_event { u32 time, s16 value, u8 type, u8 number }
                ev = os.read(js_fd, 8)
                if not ev or len(ev) < 8:
                    time.sleep(0.01)
                    continue

                _, val, ev_type, num = struct.unpack('IhBB', ev)

                # Axis event
                if ev_type & 0x02:
                    norm_val = val / 32767.0
                    raw_axes[num] = norm_val

                    raw_lx = raw_axes.get(yaw_axis, 0.0)
                    raw_ly = raw_axes.get(thr_axis, 0.0)
                    raw_rx = raw_axes.get(roll_axis, 0.0)
                    raw_ry = raw_axes.get(pitch_axis, 0.0)

                    def dz(v):
                        if abs(v) < DEADZONE: return 0.0
                        return (1.0 if v > 0 else -1.0) * ((abs(v) - DEADZONE) / (1.0 - DEADZONE))

                    norm_lx = dz(raw_lx)
                    norm_ly = dz(raw_ly)
                    norm_rx = dz(raw_rx)
                    norm_ry = dz(raw_ry)

                    # Update velocities from sticks (stick UP is negative raw, maps to +Forward/+Climb)
                    j_vx = -norm_ry * self.speed
                    j_vy = -norm_rx * self.speed
                    j_vz = -norm_ly * self.vertical_speed
                    j_wz = -norm_lx * self.yaw_speed

                    is_moving = (abs(j_vx) > 0.02 or abs(j_vy) > 0.02 or abs(j_vz) > 0.02 or abs(j_wz) > 0.02)
                    if is_moving:
                        self.vx = j_vx
                        self.vy = j_vy
                        self.vz = j_vz
                        self.wz = j_wz
                        self.js_active = True
                        print(
                            f"\r[🎮 8BitDo Stick] vx={self.vx:+.2f} vy={self.vy:+.2f} vz={self.vz:+.2f} yaw={self.wz:+.2f} | Speed: {self.speed:.1f} m/s   ",
                            end='',
                            flush=True
                        )
                    elif self.js_active:
                        self.vx = 0.0
                        self.vy = 0.0
                        self.vz = 0.0
                        self.wz = 0.0
                        self.js_active = False
                        self.publish_command()
                        print(
                            f"\r[🎮 8BitDo Centered] Drone Position Hold Locked                                    ",
                            end='',
                            flush=True
                        )

                # Button event
                elif ev_type & 0x01:
                    pressed = bool(val)
                    was_pressed = raw_buttons.get(num, False)
                    raw_buttons[num] = pressed

                    if pressed and not was_pressed:
                        # 8BitDo buttons: 3=Y, 0=A, 2=X, 1=B, 4=LB, 5=RB
                        if num == 3:
                            self.call_mode_guided()
                            print("\r[🎮 8BitDo Button Y] Mode -> GUIDED (Next: Press A to ARM)           ", flush=True)
                        elif num == 0:
                            self.call_arm(True)
                            print("\r[🎮 8BitDo Button A] ARM MOTORS -> Press Button X (TAKEOFF 5M) to Fly! ", flush=True)
                        elif num == 2:
                            self.call_takeoff(5.0)
                            print("\r[🎮 8BitDo Button X] TAKEOFF 5M -> Drone climbing, sticks engage at 5m!", flush=True)
                        elif num == 1:
                            self.vx = 0.0; self.vy = 0.0; self.vz = 0.0; self.wz = 0.0
                            self.publish_command()
                            print("\r[🎮 8BitDo Button B] BRAKE / HOVER LOCKED                            ", flush=True)
                        elif num == 4:
                            self.speed = 0.5
                            print(f"\r[🎮 8BitDo LB] Precision Speed: {self.speed:.1f} m/s          ", flush=True)
                        elif num == 5:
                            self.speed = 2.0
                            print(f"\r[🎮 8BitDo RB] Turbo Speed: {self.speed:.1f} m/s              ", flush=True)

            except BlockingIOError:
                time.sleep(0.01)
            except Exception as e:
                if js_fd:
                    try: os.close(js_fd)
                    except Exception: pass
                js_fd = None
                time.sleep(1.0)

        if js_fd:
            try: os.close(js_fd)
            except Exception: pass

    # =========================================================================
    # Keyboard Thread
    # =========================================================================
    def read_key(self):
        """Read standard keys and 3-byte ANSI arrow key escape sequences."""
        if not select.select([sys.stdin], [], [], 0.05)[0]:
            return None

        ch = sys.stdin.read(1)
        if ch == '\x1b':
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    if select.select([sys.stdin], [], [], 0.05)[0]:
                        ch3 = sys.stdin.read(1)
                        if ch3 == 'A': return 'KEY_UP'
                        elif ch3 == 'B': return 'KEY_DOWN'
                        elif ch3 == 'C': return 'KEY_RIGHT'
                        elif ch3 == 'D': return 'KEY_LEFT'
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

                # --- 1. Standard Default ROS teleop_twist_keyboard Layout ---
                if key == 'i' or key == 'KEY_UP' or key == 'w':
                    self.vx = self.speed
                    self.vy = 0.0
                    action = "▲ FORWARD"
                elif key == ',' or key == 'KEY_DOWN' or key == 's':
                    self.vx = -self.speed
                    self.vy = 0.0
                    action = "▼ BACKWARD"
                elif key == 'j':
                    self.wz = self.yaw_speed
                    action = "↺ YAW LEFT"
                elif key == 'l':
                    self.wz = -self.yaw_speed
                    action = "↻ YAW RIGHT"
                elif key == 'u':
                    self.vx = self.speed
                    self.wz = self.yaw_speed
                    action = "↖ FWD + YAW L"
                elif key == 'o':
                    self.vx = self.speed
                    self.wz = -self.yaw_speed
                    action = "↗ FWD + YAW R"
                elif key == 'm':
                    self.vx = -self.speed
                    self.wz = self.yaw_speed
                    action = "↙ BACK + YAW L"
                elif key == '.':
                    self.vx = -self.speed
                    self.wz = -self.yaw_speed
                    action = "↘ BACK + YAW R"

                # --- 2. Holonomic Strafe Keys (Shift + Keys or Arrow/AD) ---
                elif key == 'J' or key == 'KEY_LEFT' or key == 'a':
                    self.vy = self.speed
                    self.vx = 0.0
                    action = "◄ STRAFE LEFT"
                elif key == 'L' or key == 'KEY_RIGHT' or key == 'd':
                    self.vy = -self.speed
                    self.vx = 0.0
                    action = "► STRAFE RIGHT"
                elif key == 'I':
                    self.vx = self.speed
                    self.vy = 0.0
                    action = "▲ FORWARD"
                elif key == '<':
                    self.vx = -self.speed
                    self.vy = 0.0
                    action = "▼ BACKWARD"
                elif key == 'U':
                    self.vx = self.speed
                    self.vy = self.speed
                    action = "↖ FWD + STRAFE L"
                elif key == 'O':
                    self.vx = self.speed
                    self.vy = -self.speed
                    action = "↗ FWD + STRAFE R"
                elif key == 'M':
                    self.vx = -self.speed
                    self.vy = self.speed
                    action = "↙ BACK + STRAFE L"
                elif key == '>':
                    self.vx = -self.speed
                    self.vy = -self.speed
                    action = "↘ BACK + STRAFE R"

                # --- 3. Altitude (Climb Up / Descend) ---
                elif key == 't':
                    self.vz = self.vertical_speed
                    action = "▲ CLIMB UP"
                elif key == 'b':
                    self.vz = -self.vertical_speed
                    action = "▼ CLIMB DOWN"

                # --- 4. Stop / Hover Hold ---
                elif key in ('k', 'K', ' ', 'h', 'H'):
                    self.vx = 0.0
                    self.vy = 0.0
                    self.vz = 0.0
                    self.wz = 0.0
                    action = "■ HOVER / STOP"
                    self.publish_command()

                # --- 5. Speed Tuning ---
                elif key == 'q':
                    self.speed = min(5.0, round(self.speed * 1.1, 2))
                    self.yaw_speed = min(2.5, round(self.yaw_speed * 1.1, 2))
                    action = f"SPEED+ ({self.speed:.2f} m/s)"
                elif key == 'z':
                    self.speed = max(0.2, round(self.speed * 0.9, 2))
                    self.yaw_speed = max(0.1, round(self.yaw_speed * 0.9, 2))
                    action = f"SPEED- ({self.speed:.2f} m/s)"
                elif key in ('+', '='):
                    self.speed = min(5.0, round(self.speed + 0.2, 1))
                    action = f"SPEED+ ({self.speed:.1f} m/s)"
                elif key in ('-', '_'):
                    self.speed = max(0.2, round(self.speed - 0.2, 1))
                    action = f"SPEED- ({self.speed:.1f} m/s)"

                # --- 6. Vehicle Actions ---
                elif key in ('y', 'Y', 'g', 'G'):
                    self.call_mode_guided()
                    action = "MODE -> GUIDED"
                elif key in ('1', 'r', 'R'):
                    self.call_arm(True)
                    action = "ARM MOTORS"
                elif key in ('0',):
                    self.call_arm(False)
                    action = "DISARM MOTORS"
                elif key in ('x', 'X'):
                    self.call_takeoff(5.0)
                    action = "TAKEOFF 5M"

                print(
                    f"\r[ACT: {action:<18}] vx={self.vx:+.2f} vy={self.vy:+.2f} vz={self.vz:+.2f} yaw={self.wz:+.2f} | Speed: {self.speed:.1f} m/s   ",
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

        # Always publish to /drones_controller/teleop_cmd for controller_node arbitration
        self.teleop_pub.publish(msg)

        # Fallback to direct /ap/v1/cmd_vel if controller_node is not running
        if self.teleop_pub.get_subscription_count() == 0:
            self.cmd_pub.publish(msg)


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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
