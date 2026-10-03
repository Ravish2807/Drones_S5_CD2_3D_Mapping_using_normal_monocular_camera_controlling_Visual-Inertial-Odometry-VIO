#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Header
import tf2_ros
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

from interfaces.vio_interface import VIOInterface
from trajectory.camera_pose import (
    BodyPose,
    CameraExtrinsics,
    CameraPose,
    CameraTrajectory,
    transform_body_to_camera_pose,
)
from trajectory.trajectory_exporter import TrajectoryExporter


class VIONode(Node):
    def __init__(self):
        super().__init__('vio_estimator_node')
        
        # Declare parameters
        self.declare_parameter('config_path', 'config/project_vio.yaml')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        # Resolve config path using ament package share if relative
        if not os.path.isabs(config_path):
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('vio_estimator')
                candidate = os.path.join(pkg_share, config_path)
                if os.path.exists(candidate):
                    config_path = candidate
                elif os.path.exists(os.path.join(pkg_share, 'project_vio.yaml')):
                    config_path = os.path.join(pkg_share, 'project_vio.yaml')
            except Exception:
                pass

        self.get_logger().info(f"Initializing MSCKF VIO Node with config: {config_path}")
        self.vio_interface = VIOInterface(config_path)
        self.bridge = CvBridge()
        self.extrinsics = CameraExtrinsics.from_config(self.vio_interface.config)
        self.camera_traj = CameraTrajectory(name="live_camera_trajectory")

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/drone/vio/odometry', 10)
        self.path_pub = self.create_publisher(Path, '/drone/vio/path', 10)
        self.cam_pose_pub = self.create_publisher(PoseStamped, '/drone/vio/camera_pose', 10)
        self.cam_path_pub = self.create_publisher(Path, '/drone/vio/camera_path', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/drone/vio/pointcloud', 10)
        self.overlay_pub = self.create_publisher(Image, '/camera/vio_overlay', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Retained path trajectory history
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'
        self.cam_path_msg = Path()
        self.cam_path_msg.header.frame_id = 'world'

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 100
        )
        self.cam_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )

        # Periodic auto-save every 5 seconds so live results are always saved
        self.auto_save_timer = self.create_timer(5.0, self.auto_save_callback)

    def auto_save_callback(self):
        if len(self.camera_traj) > 5:
            self.save_scanned_map(silent=True)
        self.get_logger().info("VIO Estimator Node fully initialized!")
        self.get_logger().info("Publishing: /drone/vio/odometry, /drone/vio/path, /drone/vio/camera_pose, /drone/vio/camera_path, /drone/vio/pointcloud, /camera/vio_overlay")

    def imu_callback(self, msg: Imu):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        self.vio_interface.push_imu(t_sec, ax, ay, az, gx, gy, gz)

    def camera_callback(self, msg: Image):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        state = self.vio_interface.push_camera_frame(t_sec, cv_img)

        # Publish feature tracking visualization overlay
        self.publish_camera_overlay(msg.header, cv_img, state)

        if state is not None:
            self.publish_odometry_and_map(msg.header.stamp, state)

    def publish_camera_overlay(self, header: Header, cv_img: np.ndarray, state):
        """Draws tracked 2D visual features on the camera feed."""
        color_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        tracker = self.vio_interface.estimator.tracker

        if tracker.prev_pts is not None and len(tracker.prev_pts) > 0:
            for pt in tracker.prev_pts:
                cv2.circle(color_img, (int(round(pt[0])), int(round(pt[1]))), 4, (0, 255, 0), -1)

        # Overlay VIO Status text
        status_str = f"VIO State: {'OK' if state is not None else 'Initializing'}"
        cv2.putText(color_img, status_str, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if state is not None:
            feat_str = f"Tracked Features: {state.active_features} | Clones: {state.clones_count}"
            cv2.putText(color_img, feat_str, (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        overlay_msg = self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
        overlay_msg.header = header
        self.overlay_pub.publish(overlay_msg)

    def publish_odometry_and_map(self, stamp, state):
        header = Header()
        header.stamp = stamp
        header.frame_id = 'world'

        p = state.position
        q = state.orientation_q # [qw, qx, qy, qz]
        v = state.velocity

        # 1. Publish Odometry message
        odom = Odometry()
        odom.header = header
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = float(p[0])
        odom.pose.pose.position.y = float(p[1])
        odom.pose.pose.position.z = float(p[2])

        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])

        odom.twist.twist.linear.x = float(v[0])
        odom.twist.twist.linear.y = float(v[1])
        odom.twist.twist.linear.z = float(v[2])

        self.odom_pub.publish(odom)

        # 2. Append & Publish 3D Drone Body Trajectory Path
        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.pose = odom.pose.pose
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose_stamped)
        self.path_pub.publish(self.path_msg)

        # 3. Phase 2: Compute & Publish Camera Pose T_WC = T_WB * T_BC
        body_pose = BodyPose.from_vio_state(state)
        cam_pose = transform_body_to_camera_pose(body_pose, self.extrinsics)
        self.camera_traj.append(cam_pose)

        cam_pose_stamped = PoseStamped()
        cam_pose_stamped.header = header
        cam_pose_stamped.pose.position.x = float(cam_pose.p_WC[0])
        cam_pose_stamped.pose.position.y = float(cam_pose.p_WC[1])
        cam_pose_stamped.pose.position.z = float(cam_pose.p_WC[2])
        cam_q = cam_pose.q_xyzw  # [x, y, z, w]
        cam_pose_stamped.pose.orientation.x = float(cam_q[0])
        cam_pose_stamped.pose.orientation.y = float(cam_q[1])
        cam_pose_stamped.pose.orientation.z = float(cam_q[2])
        cam_pose_stamped.pose.orientation.w = float(cam_q[3])
        self.cam_pose_pub.publish(cam_pose_stamped)

        # 4. Append & Publish 3D Camera Trajectory Path
        self.cam_path_msg.header.stamp = stamp
        self.cam_path_msg.poses.append(cam_pose_stamped)
        self.cam_path_pub.publish(self.cam_path_msg)

        # 5. Publish 3D Landmark Point Cloud (Object/Environment 3D Sketch)
        landmarks = self.vio_interface.get_landmark_cloud()
        if len(landmarks) > 0:
            cloud_msg = pc2.create_cloud_xyz32(header, landmarks.astype(np.float32))
            self.cloud_pub.publish(cloud_msg)

        # 6. Broadcast TF Transforms: world -> base_link and world -> camera_optical_frame
        t_body = TransformStamped()
        t_body.header = header
        t_body.child_frame_id = 'base_link'
        t_body.transform.translation.x = float(p[0])
        t_body.transform.translation.y = float(p[1])
        t_body.transform.translation.z = float(p[2])
        t_body.transform.rotation.w = float(q[0])
        t_body.transform.rotation.x = float(q[1])
        t_body.transform.rotation.y = float(q[2])
        t_body.transform.rotation.z = float(q[3])
        self.tf_broadcaster.sendTransform(t_body)

        t_cam = TransformStamped()
        t_cam.header = header
        t_cam.child_frame_id = 'camera_optical_frame'
        t_cam.transform.translation.x = float(cam_pose.p_WC[0])
        t_cam.transform.translation.y = float(cam_pose.p_WC[1])
        t_cam.transform.translation.z = float(cam_pose.p_WC[2])
        t_cam.transform.rotation.x = float(cam_q[0])
        t_cam.transform.rotation.y = float(cam_q[1])
        t_cam.transform.rotation.z = float(cam_q[2])
        t_cam.transform.rotation.w = float(cam_q[3])
        self.tf_broadcaster.sendTransform(t_cam)

    def save_scanned_map(self, silent=False):
        """Exports 3D point cloud scan files (.ply, .pcd) and trajectory CSV/TUM."""
        try:
            base_dir = os.path.expanduser("~/vio_ws/results")
            pc_dir = os.path.join(base_dir, "pointclouds")
            traj_dir = os.path.join(base_dir, "trajectories")
            os.makedirs(pc_dir, exist_ok=True)
            os.makedirs(traj_dir, exist_ok=True)

            ply_path = os.path.join(pc_dir, "scanned_map.ply")
            pcd_path = os.path.join(pc_dir, "scanned_map.pcd")
            body_csv_path = os.path.join(traj_dir, "vio_trajectory.csv")
            cam_csv_path = os.path.join(traj_dir, "camera_trajectory.csv")
            cam_tum_path = os.path.join(traj_dir, "camera_trajectory.tum")

            self.vio_interface.export_pointcloud_ply(ply_path)
            self.vio_interface.export_pointcloud_pcd(pcd_path)
            if len(self.vio_interface.state_history) > 0:
                self.vio_interface.export_state_csv(body_csv_path)

            if len(self.camera_traj) > 0:
                TrajectoryExporter.export_csv(self.camera_traj, cam_csv_path)
                TrajectoryExporter.export_tum(self.camera_traj, cam_tum_path)
                if not silent:
                    self.get_logger().info(f"Saved Phase 2 Camera Trajectory CSV: {cam_csv_path}")
                    self.get_logger().info(f"Saved Phase 2 Camera Trajectory TUM: {cam_tum_path}")

            if not silent:
                self.get_logger().info(f"Saved 3D scanned map PLY: {ply_path}")
                self.get_logger().info(f"Saved 3D scanned map PCD: {pcd_path}")
        except Exception as e:
            if not silent:
                self.get_logger().warn(f"Failed to export scanned map: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = VIONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_scanned_map()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
