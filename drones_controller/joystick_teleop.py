#!/usr/bin/env python3
"""
===============================================================================
8BitDo Ultimate 2C / Gamepad RC Teleoperation Node (Mode 2)
===============================================================================
Enables direct closed-loop gamepad flight for ArduPilot SITL and Gazebo.

Hardware Mapping (8BitDo Ultimate 2C / Standard Gamepad):
  [Sticks - Mode 2 Drone Standard]:
    Left Stick Vertical   : Throttle / Climb (+Z) & Descend (-Z)
    Left Stick Horizontal : Yaw Rate (+Yaw Left / -Yaw Right)
    Right Stick Vertical  : Pitch / Forward (+X) & Reverse (-X)
    Right Stick Horizontal: Roll / Strafe Left (+Y) & Strafe Right (-Y)

  [Buttons]:
    Y (Button 3, Top)     : Switch Mode to GUIDED (Mode 4)
    A (Button 0, Bottom)  : ARM Throttle (Motors)
    X (Button 2, Left)    : TAKEOFF to 5.0m
    B (Button 1, Right)   : BRAKE / Hover Hold (Zero velocity)
    LB (Button 4)         : Precision Speed (0.5 m/s)
    RB (Button 5)         : Turbo Speed (2.0 m/s)
===============================================================================
"""

import os
import sys
import time
import struct
import array
import fcntl
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped

try:
    from sensor_msgs.msg import Joy
    HAS_JOY_MSG = True
except ImportError:
    HAS_JOY_MSG = False

try:
    from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
    HAS_AP_SRV = True
except ImportError:
    HAS_AP_SRV = False

# Ensure pymavlink from ardupilot submodule is discoverable for direct MAVLink fallback
for p in ["/home/ravish/ardu_ws/src/ardupilot/modules/mavlink", "/home/ravish/ardu_ws/src/ardupilot/modules/mavlink/pymavlink"]:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)


class JoystickTeleop(Node):

    def __init__(self):
        super().__init__('joystick_teleop')

        self.declare_parameter('device', 'auto')
        self.declare_parameter('speed_normal', 1.0)
        self.declare_parameter('speed_precision', 0.5)
        self.declare_parameter('speed_turbo', 2.0)
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('rate', 50.0)

        self.speed_normal = float(self.get_parameter('speed_normal').value)
        self.speed_precision = float(self.get_parameter('speed_precision').value)
        self.speed_turbo = float(self.get_parameter('speed_turbo').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.rate = float(self.get_parameter('rate').value)

        self.max_speed = self.speed_normal
        self.speed_mode_label = "NORMAL"

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        self.raw_axes = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0}
        self.raw_buttons = {}

        self.js_device_path = None
        self.js_name = "Looking for Gamepad..."
        self.js_fd = None
        self._running = True

        ap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_pub = self.create_publisher(TwistStamped, '/ap/v1/cmd_vel', ap_qos)
        self.teleop_pub = self.create_publisher(TwistStamped, '/drones_controller/teleop_cmd', 10)

        if HAS_JOY_MSG:
            self.joy_sub = self.create_subscription(Joy, '/joy', self._on_ros_joy, 10)

        # MAVLink direct connection for reliable flight commands
        self.mavlink_conn = None
        self._init_mavlink()

        # Connect to Linux joystick device
        self._find_and_open_device()

        # Reader thread
        self.read_thread = threading.Thread(target=self._joystick_reader_loop, daemon=True)
        self.read_thread.start()

        # 50 Hz publish timer
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self._publish_loop)

        # 5 Hz terminal HUD update
        self.hud_timer = self.create_timer(0.2, self._print_hud)

        self.get_logger().info(f"8BitDo / Gamepad RC Teleoperation running at {self.rate} Hz")

    def _init_mavlink(self):
        try:
            from pymavlink import mavutil
            self.mavlink_conn = mavutil.mavlink_connection("udp:127.0.0.1:14550", timeout=1.0)
            self.get_logger().info("Connected to ArduPilot SITL via MAVLink UDP 14550")
        except Exception as e:
            self.get_logger().warn(f"MAVLink direct fallback not available: {e}")

    def _find_and_open_device(self):
        req_dev = self.get_parameter('device').value
        candidates = []
        if req_dev != 'auto' and os.path.exists(req_dev):
            candidates = [req_dev]
        else:
            # Check js1, js0, js2... prioritizing 8BitDo
            for i in range(4):
                p = f"/dev/input/js{i}"
                if os.path.exists(p):
                    candidates.append(p)

        JSIOCGNAME = 0x80006a13 + (128 << 16)
        found_path = None
        found_name = "Generic Gamepad"

        for p in candidates:
            try:
                fd = os.open(p, os.O_RDONLY | os.O_NONBLOCK)
                buf = array.array('B', [0] * 128)
                fcntl.ioctl(fd, JSIOCGNAME, buf)
                name = buf.tobytes().decode('utf-8', errors='ignore').rstrip('\x00')
                if "8BitDo" in name:
                    found_path = p
                    found_name = name
                    os.close(fd)
                    break
                elif found_path is None:
                    found_path = p
                    found_name = name
                os.close(fd)
            except Exception:
                pass

        if found_path:
            try:
                self.js_fd = os.open(found_path, os.O_RDONLY | os.O_NONBLOCK)
                self.js_device_path = found_path
                self.js_name = found_name
                self.get_logger().info(f"Opened Joystick: {self.js_name} at {self.js_device_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to open {found_path}: {e}")
        else:
            self.get_logger().warn("No Linux joystick device found in /dev/input/. Waiting or using /joy topic.")

    def _apply_deadzone(self, val):
        if abs(val) < self.deadzone:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return sign * ((abs(val) - self.deadzone) / (1.0 - self.deadzone))

    def _joystick_reader_loop(self):
        while self._running:
            if self.js_fd is None:
                time.sleep(1.0)
                self._find_and_open_device()
                continue

            try:
                # Read 8 bytes: struct js_event { u32 time, s16 value, u8 type, u8 number }
                ev = os.read(self.js_fd, 8)
                if not ev or len(ev) < 8:
                    time.sleep(0.005)
                    continue

                _, val, ev_type, num = struct.unpack('IhBB', ev)

                # Button event
                if ev_type & 0x01:
                    pressed = bool(val)
                    was_pressed = self.raw_buttons.get(num, False)
                    self.raw_buttons[num] = pressed

                    if pressed and not was_pressed:
                        self._handle_button_press(num)

                # Axis event
                elif ev_type & 0x02:
                    norm_val = val / 32767.0
                    self.raw_axes[num] = norm_val

            except BlockingIOError:
                time.sleep(0.005)
            except Exception as e:
                self.get_logger().warn(f"Joystick read error: {e}. Reopening...")
                if self.js_fd:
                    try:
                        os.close(self.js_fd)
                    except Exception:
                        pass
                self.js_fd = None
                time.sleep(1.0)

    def _handle_button_press(self, btn_index):
        # 8BitDo Ultimate 2C / Xbox layout button indices:
        # 3 = Y (Top)     -> GUIDED Mode
        # 0 = A (Bottom)  -> Arm Motors
        # 2 = X (Left)    -> Takeoff to 5m
        # 1 = B (Right)   -> Brake / Hold
        # 4 = LB          -> Precision Mode
        # 5 = RB          -> Turbo Mode

        if btn_index == 3:
            self._action_guided()
        elif btn_index == 0:
            self._action_arm()
        elif btn_index == 2:
            self._action_takeoff(5.0)
        elif btn_index == 1:
            self._action_hold()
        elif btn_index == 4:
            self.max_speed = self.speed_precision
            self.speed_mode_label = "PRECISION (0.5 m/s)"
        elif btn_index == 5:
            self.max_speed = self.speed_turbo
            self.speed_mode_label = "TURBO (2.0 m/s)"

    def _on_ros_joy(self, msg: Joy):
        # Fallback if using standard ROS 2 joy node
        if self.js_fd is not None:
            return  # Prefer direct low-latency device read

        if len(msg.axes) >= 4:
            self.raw_axes[0] = msg.axes[0]  # Left X
            self.raw_axes[1] = msg.axes[1]  # Left Y
            self.raw_axes[2] = msg.axes[2] if len(msg.axes) > 2 else 0.0  # Right X
            self.raw_axes[3] = msg.axes[3] if len(msg.axes) > 3 else 0.0  # Right Y

        for i, val in enumerate(msg.buttons):
            pressed = bool(val)
            was_pressed = self.raw_buttons.get(i, False)
            self.raw_buttons[i] = pressed
            if pressed and not was_pressed:
                self._handle_button_press(i)

    def _action_guided(self):
        self.get_logger().info("🎮 [Button Y] -> Switching to GUIDED Mode...")
        if self.mavlink_conn:
            try:
                from pymavlink import mavutil
                mode_id = self.mavlink_conn.mode_mapping().get("GUIDED", 4)
                self.mavlink_conn.mav.set_mode_send(
                    self.mavlink_conn.target_system or 1,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id
                )
                self.get_logger().info("MAVLink GUIDED mode command sent.")
            except Exception as e:
                self.get_logger().error(f"MAVLink mode switch error: {e}")

    def _action_arm(self):
        self.get_logger().info("🎮 [Button A] -> ARMING Motors...")
        if self.mavlink_conn:
            try:
                from pymavlink import mavutil
                self.mavlink_conn.mav.command_long_send(
                    self.mavlink_conn.target_system or 1,
                    self.mavlink_conn.target_component or 1,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1, 0, 0, 0, 0, 0, 0
                )
                self.get_logger().info("MAVLink ARM motors command sent.")
            except Exception as e:
                self.get_logger().error(f"MAVLink arming error: {e}")

    def _action_takeoff(self, altitude=5.0):
        self.get_logger().info(f"🎮 [Button X] -> TAKEOFF to {altitude:.1f}m...")
        if self.mavlink_conn:
            try:
                from pymavlink import mavutil
                self.mavlink_conn.mav.command_long_send(
                    self.mavlink_conn.target_system or 1,
                    self.mavlink_conn.target_component or 1,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0,
                    0, 0, 0, 0, 0, 0,
                    float(altitude)
                )
                self.get_logger().info(f"MAVLink TAKEOFF ({altitude:.1f}m) command sent.")
            except Exception as e:
                self.get_logger().error(f"MAVLink takeoff error: {e}")

    def _action_hold(self):
        self.get_logger().info("🎮 [Button B] -> BRAKE / HOVER HOLD...")
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.teleop_pub.publish(msg)
        if self.teleop_pub.get_subscription_count() == 0:
            self.cmd_pub.publish(msg)

    def _publish_loop(self):
        # RC Mode 2 Mapping:
        # Left Stick: Axis 1 = Throttle (Up is -1, Down is +1), Axis 0 = Yaw (Left is -1, Right is +1)
        # Right Stick: Axis 3 = Pitch (Up is -1, Down is +1), Axis 2 = Roll (Left is -1, Right is +1)
        raw_left_x = self.raw_axes.get(0, 0.0)
        raw_left_y = self.raw_axes.get(1, 0.0)
        raw_right_x = self.raw_axes.get(2, 0.0)
        raw_right_y = self.raw_axes.get(3, 0.0)

        # Check LB / RB speed modifier
        lb = self.raw_buttons.get(4, False)
        rb = self.raw_buttons.get(5, False)
        if lb and not rb:
            speed = self.speed_precision
            self.speed_mode_label = "PRECISION (0.5 m/s)"
        elif rb and not lb:
            speed = self.speed_turbo
            self.speed_mode_label = "TURBO (2.0 m/s)"
        else:
            speed = self.speed_normal
            self.speed_mode_label = "NORMAL (1.0 m/s)"

        norm_lx = self._apply_deadzone(raw_left_x)
        norm_ly = self._apply_deadzone(raw_left_y)
        norm_rx = self._apply_deadzone(raw_right_x)
        norm_ry = self._apply_deadzone(raw_right_y)

        # Invert vertical axes: pushing stick UP is negative raw, which maps to positive forward/climb
        self.vx = -norm_ry * speed
        self.vy = -norm_rx * speed
        self.vz = -norm_ly * 0.8
        self.yaw_rate = -norm_lx * 0.8

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(self.vx)
        msg.twist.linear.y = float(self.vy)
        msg.twist.linear.z = float(self.vz)
        msg.twist.angular.z = float(self.yaw_rate)

        # Always publish to teleop topic for controller_node arbitration
        self.teleop_pub.publish(msg)

        # Fallback to direct /ap/v1/cmd_vel only when controller_node is not subscribed
        if self.teleop_pub.get_subscription_count() == 0:
            self.cmd_pub.publish(msg)

    def _print_hud(self):
        def bar(val, max_val=2.0, length=8):
            clamped = max(-max_val, min(max_val, val))
            pos = int((clamped + max_val) / (2.0 * max_val) * length)
            chars = ['-'] * length
            if 0 <= pos < length:
                chars[pos] = '#'
            return ''.join(chars)

        y_str = "[Y]" if self.raw_buttons.get(3, False) else " Y "
        a_str = "[A]" if self.raw_buttons.get(0, False) else " A "
        x_str = "[X]" if self.raw_buttons.get(2, False) else " X "
        b_str = "[B]" if self.raw_buttons.get(1, False) else " B "

        hud = (
            f"\r🎮 [{self.js_name[:20]}] | {self.speed_mode_label} | "
            f"Vx:{self.vx:+5.2f} [{bar(self.vx)}] Vy:{self.vy:+5.2f} | "
            f"Vz:{self.vz:+5.2f} [{bar(self.vz)}] Yaw:{self.yaw_rate:+5.2f} | "
            f"{y_str}Guided {a_str}Arm {x_str}Takeoff(5m) {b_str}Brake   "
        )
        sys.stdout.write(hud)
        sys.stdout.flush()

    def destroy_node(self):
        self._running = False
        if self.js_fd:
            try:
                os.close(self.js_fd)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\n[JoystickTeleop] Shutdown cleanly.")


if __name__ == '__main__':
    main()
