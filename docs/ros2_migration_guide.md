# ROS 2 Deployment & Migration Guide (Ubuntu 22.04 + ROS 2 Humble)

This guide details how to transfer and execute this standalone VIO state estimator on another machine (Ubuntu 22.04 / drone companion computer) connected to live hardware cameras, IMUs, or Gazebo ROS 2 topics.

---

## 1. Transferring Codebase to Target Machine

```bash
# On your Linux machine or drone companion computer:
git clone <your-repo-url> vio_ws/src/vio_estimator
cd vio_ws/src/vio_estimator

# Install Python scientific dependencies
pip install -r requirements.txt
```

---

## 2. Standard ROS 2 Node Integration Example

Save this script as `ros2_nodes/vio_node.py` in your ROS 2 package:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from cv_bridge import CvBridge
import numpy as np

from src.estimator_wrapper.vio_interface import VIOStreamInterface


class VIONode(Node):
    def __init__(self):
        super().__init__('vio_estimator_node')
        
        # Declare parameters
        self.declare_parameter('config_path', 'config/project_vio.yaml')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        self.get_logger().info(f"Initializing MSCKF VIO with config: {config_path}")
        self.vio_interface = VIOStreamInterface(config_path)
        self.bridge = CvBridge()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/drone/vio/odometry', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 100
        )
        self.cam_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )

    def imu_callback(self, msg: Imu):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        self.vio_interface.push_imu(t_sec, accel, gyro)

    def camera_callback(self, msg: Image):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        odom_data = self.vio_interface.push_camera_frame(t_sec, cv_img)
        if odom_data is not None:
            self.publish_odometry(msg.header.stamp, odom_data)

    def publish_odometry(self, stamp, odom_data):
        # 1. Publish Odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'

        p = odom_data['position']
        q = odom_data['orientation_q'] # [w, x, y, z]
        v = odom_data['velocity']

        odom.pose.pose.position.x = p[0]
        odom.pose.pose.position.y = p[1]
        odom.pose.pose.position.z = p[2]

        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]

        odom.twist.twist.linear.x = v[0]
        odom.twist.twist.linear.y = v[1]
        odom.twist.twist.linear.z = v[2]

        self.odom_pub.publish(odom)

        # 2. Broadcast TF Transform
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = p[0]
        t.transform.translation.y = p[1]
        t.transform.translation.z = p[2]
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = VIONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 3. Topic Mapping Conventions

| Sensor Stream | Default ROS 2 Topic | Type | Rate |
| :--- | :--- | :--- | :--- |
| **IMU** | `/imu/data` or `/camera/imu` | `sensor_msgs/msg/Imu` | 200 Hz |
| **Camera** | `/camera/image_raw` | `sensor_msgs/msg/Image` | 20–30 Hz |
| **VIO Odometry Output** | `/drone/vio/odometry` | `nav_msgs/msg/Odometry` | 20–30 Hz |
| **TF Frame** | `world -> base_link` | `tf2_msgs/msg/TFMessage` | 20–30 Hz |
