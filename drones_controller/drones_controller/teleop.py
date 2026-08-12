import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import select
import termios
import tty
import threading


class DroneTeleop(Node):

    def __init__(self):
        super().__init__('drone_teleop')

        self.pub = self.create_publisher(
            TwistStamped,
            '/ap/v1/cmd_vel',
            10
        )

        self.speed = 0.5
        self.yaw_speed = 0.5

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wz = 0.0

        self.running = True

        self.get_logger().info('Keyboard teleop started')

        print("""
================================
        DRONE TELEOP
================================

W : forward
S : backward
A : left
D : right

R : UP
F : DOWN

Q : yaw left
E : yaw right

SPACE : STOP
CTRL+C : EXIT

================================
""")

        self.thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True
        )

        self.thread.start()

        self.timer = self.create_timer(
            0.1,
            self.publish_command
        )

    def keyboard_loop(self):

        old_settings = termios.tcgetattr(sys.stdin)

        try:

            tty.setcbreak(sys.stdin.fileno())

            while self.running:

                if select.select(
                    [sys.stdin],
                    [],
                    [],
                    0.05
                )[0]:

                    key = sys.stdin.read(1)

                    if key == 'w':
                        self.vx = self.speed
                        self.vy = 0.0

                    elif key == 's':
                        self.vx = -self.speed
                        self.vy = 0.0

                    elif key == 'a':
                        self.vy = self.speed
                        self.vx = 0.0

                    elif key == 'd':
                        self.vy = -self.speed
                        self.vx = 0.0

                    elif key == 'r':
                        self.vz = self.speed

                    elif key == 'f':
                        self.vz = -self.speed

                    elif key == 'q':
                        self.wz = self.yaw_speed

                    elif key == 'e':
                        self.wz = -self.yaw_speed

                    elif key == ' ':
                        self.vx = 0.0
                        self.vy = 0.0
                        self.vz = 0.0
                        self.wz = 0.0

                    print(
                        f"\rCMD "
                        f"vx={self.vx:+.2f} "
                        f"vy={self.vy:+.2f} "
                        f"vz={self.vz:+.2f} "
                        f"yaw={self.wz:+.2f}",
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
        msg.header.frame_id = 'map'

        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        msg.twist.linear.z = self.vz

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = self.wz

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
