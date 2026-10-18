#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped

try:
    from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
    from ardupilot_msgs.msg import Status
    HAS_ARDUPILOT_MSGS = True
except ImportError:
    HAS_ARDUPILOT_MSGS = False

COPTER_MODE_GUIDED = 4


class ArmAndTakeoff(Node):

    def __init__(self):
        super().__init__('arm_and_takeoff')

        self.declare_parameter('alt', 3.0)
        self.target_alt = float(self.get_parameter('alt').value)

        self.current_alt = 0.0
        self.armed = False
        self.mode = 0

        if not HAS_ARDUPILOT_MSGS:
            self.get_logger().error("ardupilot_msgs package is not found! Please build and source ardupilot_msgs.")
            sys.exit(1)

        # Service clients - support both /ap/v1/ and /ap/ namespaces
        self.mode_client = self._resolve_service(ModeSwitch, ['/ap/v1/mode_switch', '/ap/mode_switch'])
        self.arm_client = self._resolve_service(ArmMotors, ['/ap/v1/arm_motors', '/ap/arm_motors'])
        self.takeoff_client = self._resolve_service(Takeoff, ['/ap/v1/experimental/takeoff', '/ap/experimental/takeoff'])

        # Subscribe to pose to monitor ascent
        qos_best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ap/v1/pose/filtered',
            self._pose_callback,
            qos_best_effort
        )

        self.get_logger().info(f"Arm & Takeoff node initialized (Target Altitude: {self.target_alt} m)")

    def _resolve_service(self, srv_type, candidates):
        for candidate in candidates:
            client = self.create_client(srv_type, candidate)
            if client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Connected to service: {candidate}")
                return client
        # Fall back to first candidate and wait
        self.get_logger().warn(f"Waiting for service: {candidates[0]} ...")
        client = self.create_client(srv_type, candidates[0])
        while not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f"Still waiting for {candidates[0]} ...")
        return client

    def _pose_callback(self, msg: PoseStamped):
        self.current_alt = msg.pose.position.z

    def run_sequence(self):
        # 1. Switch to GUIDED mode
        self.get_logger().info("1/3: Switching mode to GUIDED (mode 4)...")
        mode_req = ModeSwitch.Request()
        mode_req.mode = COPTER_MODE_GUIDED
        future = self.mode_client.call_async(mode_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        if res is not None and getattr(res, 'status', False):
            self.get_logger().info("Mode switched to GUIDED successfully.")
        else:
            self.get_logger().info(f"Mode switch response: {res}")

        time.sleep(1.0)

        # 2. Arm motors
        self.get_logger().info("2/3: Arming drone motors...")
        arm_req = ArmMotors.Request()
        arm_req.arm = True
        future = self.arm_client.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        if res is not None and getattr(res, 'result', False):
            self.get_logger().info("Motors ARMED successfully.")
        else:
            self.get_logger().info(f"ArmMotors response: {res}")

        time.sleep(1.0)

        # 3. Takeoff
        self.get_logger().info(f"3/3: Commanding takeoff to {self.target_alt} m...")
        takeoff_req = Takeoff.Request()
        takeoff_req.alt = float(self.target_alt)
        future = self.takeoff_client.call_async(takeoff_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        if res is not None and getattr(res, 'status', False):
            self.get_logger().info(f"Takeoff command accepted for {self.target_alt} m.")
        else:
            self.get_logger().info(f"Takeoff response: {res}")

        # 4. Monitor altitude
        self.get_logger().info("Monitoring ascent altitude...")
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 30.0:
            rclpy.spin_once(self, timeout_sec=0.2)
            self.get_logger().info(f"Current Altitude: {self.current_alt:.2f} m / Target: {self.target_alt:.2f} m")
            if self.current_alt >= (self.target_alt - 0.25):
                self.get_logger().info(f"Target takeoff altitude {self.target_alt}m reached successfully!")
                break
            time.sleep(0.5)

        self.get_logger().info("=" * 60)
        self.get_logger().info(" Drone is airborne and ready for autonomous flight controller!")
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = ArmAndTakeoff()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
