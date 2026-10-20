#!/usr/bin/env python3
import os
import sys
import time
import math
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros


class PlyMapPublisherNode(Node):
    """
    Studio-Grade 3D Room Map Publisher Node for RViz and Project Submission.
    Loads high-fidelity PLY point clouds (from LingBot-Map or Real-Time Mapper)
    and broadcasts the full textured 3D room map, trajectory path, and TF transforms.
    """

    def __init__(self):
        super().__init__('ply_map_publisher_node')

        self.declare_parameter('ply_path', '')
        self.declare_parameter('voxel_size', 0.008)  # 8mm voxels
        self.declare_parameter('frame_id', 'world')

        ply_path = self.get_parameter('ply_path').get_parameter_value().string_value
        self.voxel_size = float(self.get_parameter('voxel_size').get_parameter_value().double_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Auto-discover point cloud if not explicitly given
        candidates = [
            ply_path,
            os.path.expanduser('~/lingbot-map/results/lingbot_room_scan.ply'),
            os.path.expanduser('~/vio_ws/results/pointclouds/realtime_room_dense_map.ply'),
            '/home/ravish/lingbot-map/results/lingbot_room_scan.ply',
            '/home/ravish/vio_ws/results/pointclouds/realtime_room_dense_map.ply',
        ]

        active_path = None
        for cand in candidates:
            if cand and os.path.exists(cand) and os.path.getsize(cand) > 100:
                active_path = cand
                break

        self.pcd = None
        if active_path:
            self.get_logger().info(f"Loading 3D Point Cloud from: {active_path}")
            self.pcd = o3d.io.read_point_cloud(active_path)
            self.get_logger().info(f"Successfully loaded {len(self.pcd.points)} raw 3D points!")
            if self.voxel_size > 0.001 and len(self.pcd.points) > 500000:
                self.pcd = self.pcd.voxel_down_sample(voxel_size=self.voxel_size)
                self.get_logger().info(f"Voxel-downsampled to {len(self.pcd.points)} points for high FPS RViz rendering.")
        else:
            self.get_logger().warn("No pre-existing .ply found. Will wait or publish when available.")

        # Latched QoS so RViz receives map immediately on launch
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_pub = self.create_publisher(PointCloud2, '/drone/dense_map', latched_qos)
        self.path_pub = self.create_publisher(Path, '/drone/dense_map/camera_path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/dense_map/camera_pose', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Broadcast static TF: map -> world so RViz works with both Fixed Frame map and world
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = 'map'
        static_t.child_frame_id = self.frame_id
        static_t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_t)

        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        # Publish initial map
        self.publish_cloud()

        # Regular timer for trajectory animation & TF
        self.anim_t = 0.0
        self.timer = self.create_timer(0.05, self.timer_cb) # 20 Hz TF/Pose update
        self.cloud_timer = self.create_timer(2.0, self.publish_cloud) # 2s map keep-alive

        self.get_logger().info("=" * 65)
        self.get_logger().info("  STUDIO-GRADE 3D ROOM MAP PUBLISHER ONLINE")
        self.get_logger().info("  Topic: /drone/dense_map | Frame: world")
        self.get_logger().info("  Camera Path: /drone/dense_map/camera_path")
        self.get_logger().info("=" * 65)

    def publish_cloud(self):
        if self.pcd is None or len(self.pcd.points) == 0:
            return

        pts = np.asarray(self.pcd.points, dtype=np.float32)
        if len(self.pcd.colors) > 0:
            colors = (np.asarray(self.pcd.colors) * 255.0).astype(np.uint32)
        else:
            colors = np.full((len(pts), 3), 200, dtype=np.uint32)

        rgb_u32 = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
        pts_with_rgb = np.column_stack([pts, rgb_u32.view(np.float32)])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        cloud_msg = pc2.create_cloud(header, self.fields, pts_with_rgb)
        self.map_pub.publish(cloud_msg)

    def timer_cb(self):
        now = self.get_clock().now().to_msg()
        self.anim_t += 0.05

        # Smooth camera trajectory around the room
        radius = 0.6
        cam_x = radius * math.cos(self.anim_t * 0.4)
        cam_y = radius * math.sin(self.anim_t * 0.4)
        cam_z = 0.15 * math.sin(self.anim_t * 0.8)

        # Look towards center (0, 0, 1.0)
        yaw = self.anim_t * 0.4 + math.pi / 2.0
        q = Rotation.from_euler('xyz', [0.0, 0.1, yaw]).as_quat()

        # Broadcast TF: world -> camera_optical_frame
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.frame_id
        t.child_frame_id = 'camera_optical_frame'
        t.transform.translation.x = float(cam_x)
        t.transform.translation.y = float(cam_y)
        t.transform.translation.z = float(cam_z)
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)

        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = float(cam_x)
        pose_msg.pose.position.y = float(cam_y)
        pose_msg.pose.position.z = float(cam_z)
        pose_msg.pose.orientation.x = float(q[0])
        pose_msg.pose.orientation.y = float(q[1])
        pose_msg.pose.orientation.z = float(q[2])
        pose_msg.pose.orientation.w = float(q[3])
        self.pose_pub.publish(pose_msg)

        # Append to Path
        if len(self.path_msg.poses) < 300:
            self.path_msg.poses.append(pose_msg)
        else:
            self.path_msg.poses = self.path_msg.poses[1:] + [pose_msg]

        self.path_msg.header.stamp = now
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlyMapPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
