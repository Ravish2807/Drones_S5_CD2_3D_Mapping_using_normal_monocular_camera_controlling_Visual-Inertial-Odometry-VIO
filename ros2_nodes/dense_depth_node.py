#!/usr/bin/env python3
import os
import cv2
import yaml
import numpy as np
import torch
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge

try:
    from transformers import pipeline
    TRANSFORMERS_AVAILABLE = True
except ImportError:
    TRANSFORMERS_AVAILABLE = False


class DenseDepthNode(Node):
    """
    Real-Time AI-Powered Monocular Dense 3D Point Cloud Node for ROS 2.
    Converts live 2D RGB camera frames (/camera/image_raw) into a textured,
    dense 3D point cloud (/drone/vio/dense_pointcloud) using Depth-Anything-V2 on CUDA.
    """

    def __init__(self):
        super().__init__('dense_depth_node')

        # Parameters
        self.declare_parameter('config_path', 'config/project_vio.yaml')
        self.declare_parameter('min_depth', 0.25) # meters
        self.declare_parameter('max_depth', 4.5)  # meters
        self.declare_parameter('step', 4)         # point cloud subsampling step
        self.declare_parameter('publish_rate', 15.0) # max FPS

        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.min_depth = float(self.get_parameter('min_depth').get_parameter_value().double_value)
        self.max_depth = float(self.get_parameter('max_depth').get_parameter_value().double_value)
        self.step = int(self.get_parameter('step').get_parameter_value().integer_value)
        self.publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)

        # Resolve config file
        if not os.path.isabs(config_path):
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('vio_estimator')
                candidate = os.path.join(pkg_share, config_path)
                if os.path.exists(candidate):
                    config_path = candidate
                elif os.path.exists(os.path.join(pkg_share, 'config', 'project_vio.yaml')):
                    config_path = os.path.join(pkg_share, 'config', 'project_vio.yaml')
            except Exception:
                pass

        # Load intrinsics from config
        self.base_w = 640.0
        self.base_h = 480.0
        self.fx = 400.0
        self.fy = 400.0
        self.cx = 320.0
        self.cy = 240.0

        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
                cam_cfg = cfg.get('camera', {})
                self.base_w = float(cam_cfg.get('width', 640))
                self.base_h = float(cam_cfg.get('height', 480))
                intr = cam_cfg.get('intrinsics', {})
                self.fx = float(intr.get('fx', 400.0))
                self.fy = float(intr.get('fy', 400.0))
                self.cx = float(intr.get('cx', 320.0))
                self.cy = float(intr.get('cy', 240.0))

        self.get_logger().info(f"Loaded camera model: {self.base_w}x{self.base_h}, fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

        # Initialize GPU Depth Estimator
        self.device = 0 if torch.cuda.is_available() else -1
        device_name = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU"
        self.get_logger().info(f"Loading Depth-Anything-V2-Small on {device_name} (device={self.device})...")

        self.pipe = pipeline(
            task="depth-estimation",
            model="depth-anything/Depth-Anything-V2-Small-hf",
            device=self.device
        )
        self.get_logger().info("Depth-Anything-V2 Neural Model Loaded Successfully!")

        # Bridge and publishers
        self.bridge = CvBridge()
        self.cloud_pub = self.create_publisher(PointCloud2, '/drone/vio/dense_pointcloud', 5)
        self.depth_img_pub = self.create_publisher(Image, '/camera/depth/image_raw', 5)

        # Pre-allocate PointCloud2 Fields (x, y, z, rgb)
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Rate limiter
        self.last_proc_time = 0.0
        self.min_interval = 1.0 / self.publish_rate

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.get_logger().info("Dense Depth Node Listening to /camera/image_raw -> Publishing /drone/vio/dense_pointcloud")

    def image_callback(self, msg: Image):
        now = time.time()
        if (now - self.last_proc_time) < self.min_interval:
            return
        self.last_proc_time = now

        try:
            # Convert ROS Image to OpenCV BGR
            if msg.encoding in ['mono8', '8UC1']:
                cv_gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                cv_bgr = cv2.cvtColor(cv_gray, cv2.COLOR_GRAY2BGR)
            elif msg.encoding in ['rgb8']:
                cv_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                cv_bgr = cv2.cvtColor(cv_rgb, cv2.COLOR_RGB2BGR)
            else:
                cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to decode image message: {e}")
            return

        h, w, _ = cv_bgr.shape
        cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(cv_rgb)

        # 1. Run AI Depth Estimation
        t0 = time.time()
        try:
            res = self.pipe(pil_img)
            depth_raw = np.array(res['depth'], dtype=np.float32)
        except Exception as e:
            self.get_logger().warn(f"Depth model inference failed: {e}")
            return

        infer_time_ms = (time.time() - t0) * 1000.0

        # Resize depth map if needed to match input image
        if depth_raw.shape[0] != h or depth_raw.shape[1] != w:
            depth_raw = cv2.resize(depth_raw, (w, h), interpolation=cv2.INTER_LINEAR)

        # 2. Normalize and Map to Metric Range (Disparity -> Metric Depth Z)
        d_min, d_max = np.percentile(depth_raw, [2, 98])
        d_norm = np.clip((depth_raw - d_min) / (d_max - d_min + 1e-6), 0.0, 1.0)

        # Disparity to metric depth
        inv_z = (1.0 / self.max_depth) + d_norm * (1.0 / self.min_depth - 1.0 / self.max_depth)
        depth_m = 1.0 / (inv_z + 1e-6)

        # 3. Vectorized 3D Unprojection into Colored PointCloud
        step = self.step
        u_grid, v_grid = np.meshgrid(np.arange(0, w, step), np.arange(0, h, step))
        Z_grid = depth_m[v_grid, u_grid].flatten()

        valid_mask = (Z_grid >= self.min_depth) & (Z_grid <= self.max_depth)
        if not np.any(valid_mask):
            return

        u_v = u_grid.flatten()[valid_mask]
        v_v = v_grid.flatten()[valid_mask]
        Z_v = Z_grid[valid_mask]

        # Scale intrinsics to input image resolution
        scale_x = float(w) / self.base_w
        scale_y = float(h) / self.base_h
        fx = self.fx * scale_x
        fy = self.fy * scale_y
        cx = self.cx * scale_x
        cy = self.cy * scale_y

        # Camera frame (+Z forward, +X right, +Y down)
        X_v = (u_v - cx) * Z_v / fx
        Y_v = (v_v - cy) * Z_v / fy

        # Color extraction from RGB
        rgb_sampled = cv_rgb[v_v, u_v] # (N, 3)
        r = rgb_sampled[:, 0].astype(np.uint32)
        g = rgb_sampled[:, 1].astype(np.uint32)
        b = rgb_sampled[:, 2].astype(np.uint32)
        rgb_u32 = (r << 16) | (g << 8) | b
        rgb_float = rgb_u32.view(np.float32)

        pts = np.column_stack([X_v, Y_v, Z_v, rgb_float]).astype(np.float32)

        # 4. Publish PointCloud2 in camera_optical_frame
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = 'camera_optical_frame'

        cloud_msg = pc2.create_cloud(header, self.fields, pts)
        self.cloud_pub.publish(cloud_msg)

        # 5. Publish Colorized Depth Map
        depth_vis = (d_norm * 255.0).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_color, encoding='bgr8')
        depth_msg.header = header
        self.depth_img_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DenseDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
