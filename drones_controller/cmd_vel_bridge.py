import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelBridge(Node):
    """
    Bridges standard ROS unstamped /cmd_vel (Twist, Reliable QoS)
    to ArduPilot AP_DDS /ap/v1/cmd_vel (TwistStamped, base_link frame, Best Effort QoS).
    """

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/ap/v1/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        ap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            Twist,
            input_topic,
            self.twist_callback,
            10
        )

        self.pub = self.create_publisher(
            TwistStamped,
            output_topic,
            ap_qos
        )

        self.get_logger().info(
            f"Bridge initialized: Subscribing to '{input_topic}' (Twist) -> Publishing to '{output_topic}' (TwistStamped, frame='{self.frame_id}')"
        )

    def twist_callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
