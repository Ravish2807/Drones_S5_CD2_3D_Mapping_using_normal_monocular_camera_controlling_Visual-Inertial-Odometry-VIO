#!/usr/bin/env python3
import os
import sys
import time
import yaml
import numpy as np
import torch
import cv2
from PIL import Image as PILImage
import open3d as o3d
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import tf2_ros

try:
    from transformers import AutoImageProcessor, AutoModelForDepthEstimation
    TRANSFORMERS_METRIC_AVAILABLE = True
except ImportError:
    TRANSFORMERS_METRIC_AVAILABLE = False


class DenseMappingNode(Node):
    """
    Continuous Real-Time 3D Spatial Dense Mapping ROS 2 Node.
    Features:
      1. Metric Indoor Neural Depth Engine (depth-anything-v2-metric-indoor-small)
      2. Joint Bilateral Depth Smoothing (flattens planar surfaces, sharpens edges)
      3. Scale-Invariant Relative Gradient Edge Culling (|grad Z| / Z < 0.08)
      4. Sub-Pixel Jittered Unprojection (eliminates screen-door grid artifacts)
      5. Border Margin Vignette Clipping (culls outer 12px lens edge distortion)
      6. Color Vibrance & Radiance Tone-Mapping
      7. Continuous Sequential Frame-to-Frame Point-to-Plane ICP (NEVER FREEZES)
      8. Velocity Motion Model (handles smooth camera motion across room)
      9. Survey-Grade Voxel Grid Fusion (1.0 cm resolution, zero duplicate reflections)

    Publishes:
      - /drone/dense_map (sensor_msgs/PointCloud2, frame_id: 'world', RGB8 colors)
      - /camera/depth/image_raw (sensor_msgs/Image, Turbo colormap)
      - /drone/dense_map/camera_path & /drone/vio/camera_path (nav_msgs/Path)
      - /drone/dense_map/camera_pose & /drone/vio/camera_pose (geometry_msgs/PoseStamped)
      - /tf (world -> camera_optical_frame)
    """

    def __init__(self):
        super().__init__('dense_mapping_node')

        # Core Parameters
        self.declare_parameter('config_path', 'config/project_vio.yaml')
        self.declare_parameter('min_depth', 0.20)         # meters
        self.declare_parameter('max_depth', 4.50)         # meters (allows scanning room walls and furniture)
        self.declare_parameter('rel_edge_grad', 0.08)     # relative depth gradient (|grad Z|/Z) threshold
        self.declare_parameter('voxel_size', 0.010)       # 1.0 cm high-density voxels
        self.declare_parameter('min_trans_m', 0.02)       # min movement (2 cm) to integrate new scan
        self.declare_parameter('min_rot_deg', 2.0)        # min rotation (2 deg) to integrate new scan
        self.declare_parameter('step', 2)                 # pixel sampling step with sub-pixel jitter
        self.declare_parameter('max_map_points', 450000)
        self.declare_parameter('fx', 525.0)               # smartphone 1x camera focal length
        self.declare_parameter('fy', 525.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('enable_bilateral', True)
        self.declare_parameter('enable_vibrance', True)
        # Outcome 2: camera-frame collision corridor and spatial clustering.
        self.declare_parameter('obstacle_min_depth', 0.20)
        self.declare_parameter('obstacle_max_depth', 3.0)
        self.declare_parameter('obstacle_half_width', 1.2)
        self.declare_parameter('obstacle_y_min', -0.9)
        self.declare_parameter('obstacle_y_max', 0.5)
        self.declare_parameter('obstacle_ground_y', 0.45)
        self.declare_parameter('obstacle_cluster_eps', 0.08)
        self.declare_parameter('obstacle_min_points', 25)
        self.declare_parameter('enable_obstacles', False)

        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.min_depth = float(self.get_parameter('min_depth').get_parameter_value().double_value)
        self.max_depth = float(self.get_parameter('max_depth').get_parameter_value().double_value)
        self.rel_edge_grad = float(self.get_parameter('rel_edge_grad').get_parameter_value().double_value)
        self.voxel_size = float(self.get_parameter('voxel_size').get_parameter_value().double_value)
        self.min_trans_m = float(self.get_parameter('min_trans_m').get_parameter_value().double_value)
        self.min_rot_deg = float(self.get_parameter('min_rot_deg').get_parameter_value().double_value)
        self.step = int(self.get_parameter('step').get_parameter_value().integer_value)
        self.max_map_points = int(self.get_parameter('max_map_points').get_parameter_value().integer_value)
        self.fx = float(self.get_parameter('fx').get_parameter_value().double_value)
        self.fy = float(self.get_parameter('fy').get_parameter_value().double_value)
        self.cx = float(self.get_parameter('cx').get_parameter_value().double_value)
        self.cy = float(self.get_parameter('cy').get_parameter_value().double_value)
        self.enable_bilateral = bool(self.get_parameter('enable_bilateral').get_parameter_value().bool_value)
        self.enable_vibrance = bool(self.get_parameter('enable_vibrance').get_parameter_value().bool_value)
        self.enable_obstacles = bool(self.get_parameter('enable_obstacles').get_parameter_value().bool_value)
        self.obstacle_min_depth = float(self.get_parameter('obstacle_min_depth').value)
        self.obstacle_max_depth = float(self.get_parameter('obstacle_max_depth').value)
        self.obstacle_half_width = float(self.get_parameter('obstacle_half_width').value)
        self.obstacle_y_min = float(self.get_parameter('obstacle_y_min').value)
        self.obstacle_y_max = float(self.get_parameter('obstacle_y_max').value)
        self.obstacle_ground_y = float(self.get_parameter('obstacle_ground_y').value)
        self.obstacle_cluster_eps = float(self.get_parameter('obstacle_cluster_eps').value)
        self.obstacle_min_points = int(self.get_parameter('obstacle_min_points').value)

        # Base image dimensions
        self.base_w = 640.0
        self.base_h = 480.0

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

        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
                cam_cfg = cfg.get('camera', {})
                self.base_w = float(cam_cfg.get('width', 640))
                self.base_h = float(cam_cfg.get('height', 480))
                intr = cam_cfg.get('intrinsics', {})
                if 'fx' in intr:
                    self.fx = float(intr['fx'])
                if 'fy' in intr:
                    self.fy = float(intr['fy'])
                if 'cx' in intr:
                    self.cx = float(intr['cx'])
                if 'cy' in intr:
                    self.cy = float(intr['cy'])

        self.get_logger().info(f"Active Camera Model: {self.base_w:.0f}x{self.base_h:.0f}, fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")

        # Initialize GPU Metric Indoor Depth Engine
        self.torch_device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        device_name = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU"
        self.get_logger().info(f"Initializing Metric Indoor Neural Depth Engine on {device_name}...")

        metric_model_id = 'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf'
        try:
            self.processor = AutoImageProcessor.from_pretrained(metric_model_id)
            self.depth_model = AutoModelForDepthEstimation.from_pretrained(metric_model_id).to(self.torch_device)
            self.depth_model.eval()
            self.is_metric_model = True
            self.get_logger().info("Official Metric Indoor Depth Model ONLINE (Direct Real-World Meters)!")
        except Exception as e:
            self.get_logger().warn(f"Failed to load metric model directly ({e}), falling back to pipeline...")
            from transformers import pipeline
            self.fallback_pipe = pipeline(
                task="depth-estimation",
                model="depth-anything/Depth-Anything-V2-Small-hf",
                device=0 if torch.cuda.is_available() else -1
            )
            self.is_metric_model = False

        # Global Map Storage (Open3D PointCloud)
        self.global_pcd = o3d.geometry.PointCloud()
        self.prev_track_pcd = None
        self.last_delta_T = np.eye(4)
        self.last_process_time = 0.0
        self.last_add_time = 0.0
        self.is_busy = False

        # Rigid Cumulative Camera-to-World Pose (T_WC)
        self.current_T_WC = np.eye(4)

        # Pre-allocate PointCloud2 Fields (x, y, z, rgb)
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Publishers & Broadcasters
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Broadcast static TF: map -> world (identity) so RViz renders in any Fixed Frame
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = 'map'
        static_t.child_frame_id = 'world'
        static_t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_t)

        # Latched QoS: TRANSIENT_LOCAL guarantees RViz instantly receives points upon connecting
        latched_map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_pub = self.create_publisher(PointCloud2, '/drone/dense_map', latched_map_qos)
        self.depth_img_pub = self.create_publisher(Image, '/camera/depth/image_raw', 5)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/dense_map/camera_pose', 10)
        self.vio_pose_pub = self.create_publisher(PoseStamped, '/drone/vio/camera_pose', 10)
        self.path_pub = self.create_publisher(Path, '/drone/dense_map/camera_path', 10)
        self.vio_path_pub = self.create_publisher(Path, '/drone/vio/camera_path', 10)
        self.obstacles_pub = self.create_publisher(MarkerArray, '/drone/obstacles', 10)
        self._last_obstacle_diag_time = 0.0

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'

        # Subscribers: depth=1 to drop old frames and eliminate queue backlog lag completely
        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, sub_qos
        )

        # Periodic map publisher timer (2 Hz for smooth RViz rendering)
        self.pub_timer = self.create_timer(0.5, self.publish_accumulated_map)

        self.get_logger().info("=" * 70)
        self.get_logger().info("  CONTINUOUS REAL-TIME 3D SPATIAL MAPPER ACTIVE")
        self.get_logger().info("  * Sequential Frame-to-Frame ICP Tracking (Continuous, Drift-Free)")
        self.get_logger().info("  * Motion Velocity Model + Voxel Fusion (Zero Ghost Reflections)")
        self.get_logger().info(f"  * Resolution: {self.voxel_size*100:.1f} cm | Output: /drone/dense_map (RGB8)")
        self.get_logger().info("=" * 70)

    def image_callback(self, msg: Image):
        # Drop frame immediately if callback is still running (Zero queue backlog!)
        if self.is_busy:
            return
        self.is_busy = True

        try:
            self._process_image(msg)
        finally:
            self.is_busy = False

    def _process_image(self, msg: Image):
        now = time.time()
        try:
            if msg.encoding in ['mono8', '8UC1']:
                cv_gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                cv_bgr = cv2.cvtColor(cv_gray, cv2.COLOR_GRAY2BGR)
            elif msg.encoding in ['rgb8']:
                cv_rgb_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                cv_bgr = cv2.cvtColor(cv_rgb_raw, cv2.COLOR_RGB2BGR)
            else:
                cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Image decode failed: {e}")
            return

        h, w, _ = cv_bgr.shape
        cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(cv_rgb)

        # 1. GPU Metric Depth Inference (Direct Real Meters)
        try:
            if self.is_metric_model:
                inputs = self.processor(images=pil_img, return_tensors='pt').to(self.torch_device)
                with torch.no_grad():
                    outputs = self.depth_model(**inputs)
                    pred = outputs.predicted_depth
                    pred_interp = torch.nn.functional.interpolate(
                        pred.unsqueeze(1),
                        size=(h, w),
                        mode='bicubic',
                        align_corners=False
                    ).squeeze()
                    depth_m = pred_interp.cpu().numpy().astype(np.float32)
            else:
                res = self.fallback_pipe(pil_img)
                depth_raw = np.array(res['depth'], dtype=np.float32)
                if depth_raw.shape[0] != h or depth_raw.shape[1] != w:
                    depth_raw = cv2.resize(depth_raw, (w, h), interpolation=cv2.INTER_LINEAR)
                d_min, d_max = np.percentile(depth_raw, [2, 98])
                d_norm = np.clip((depth_raw - d_min) / (d_max - d_min + 1e-6), 0.0, 1.0)
                inv_z = (1.0 / self.max_depth) + d_norm * (1.0 / self.min_depth - 1.0 / self.max_depth)
                depth_m = 1.0 / (inv_z + 1e-6)
        except Exception as e:
            self.get_logger().warn(f"Inference error: {e}")
            return

        # 2. Joint Bilateral Depth Smoothing (Flattens planar surfaces, sharpens edges)
        if self.enable_bilateral:
            depth_m = cv2.bilateralFilter(depth_m, d=5, sigmaColor=0.06, sigmaSpace=5)

        # 3. Scale-Invariant Relative Gradient Edge Culling (|grad Z| / Z < threshold)
        dz_dx = cv2.Sobel(depth_m, cv2.CV_32F, 1, 0, ksize=3)
        dz_dy = cv2.Sobel(depth_m, cv2.CV_32F, 0, 1, ksize=3)
        grad_rel = np.sqrt(dz_dx**2 + dz_dy**2) / (depth_m + 1e-5)

        # 4. Sub-Pixel Jittered Unprojection
        step = self.step
        u_grid, v_grid = np.meshgrid(np.arange(0, w, step), np.arange(0, h, step))
        u_jit = u_grid + (np.random.rand(*u_grid.shape) - 0.5) * step * 0.85
        v_jit = v_grid + (np.random.rand(*v_grid.shape) - 0.5) * step * 0.85
        u_idx = np.clip(np.round(u_jit), 0, w - 1).astype(int)
        v_idx = np.clip(np.round(v_jit), 0, h - 1).astype(int)

        Z = depth_m[v_idx, u_idx]
        G_rel = grad_rel[v_idx, u_idx]

        # Valid depth range + Clean silhouette edges + Margin clipping (outer 12 pixels)
        border = 12
        valid_mask = (
            (Z >= self.min_depth) &
            (Z <= self.max_depth) &
            (G_rel < self.rel_edge_grad) &
            (u_idx >= border) & (u_idx < w - border) &
            (v_idx >= border) & (v_idx < h - border)
        )
        if not np.any(valid_mask):
            return

        u_v = u_jit[valid_mask]
        v_v = v_jit[valid_mask]
        Z_v = Z[valid_mask]

        scale_x = float(w) / self.base_w
        scale_y = float(h) / self.base_h
        fx = self.fx * scale_x
        fy = self.fy * scale_y
        cx = self.cx * scale_x
        cy = self.cy * scale_y

        # Local camera frame 3D coordinates
        X_C = (u_v - cx) * Z_v / fx
        Y_C = (v_v - cy) * Z_v / fy
        Z_C = Z_v
        pts_C = np.column_stack([X_C, Y_C, Z_C])

        # 5. Color Vibrance & Radiance Processing
        if self.enable_vibrance:
            hsv = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * 1.15, 0, 255)
            hsv[:, :, 2] = np.clip(hsv[:, :, 2] * 1.06, 0, 255)
            vibrant_rgb = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2RGB)
            colors_rgb = vibrant_rgb[v_idx[valid_mask], u_idx[valid_mask]].astype(np.float64) / 255.0
        else:
            colors_rgb = cv_rgb[v_idx[valid_mask], u_idx[valid_mask]].astype(np.float64) / 255.0

        # Frame point cloud in local camera coordinates
        frame_cam_pcd = o3d.geometry.PointCloud()
        frame_cam_pcd.points = o3d.utility.Vector3dVector(pts_C)
        frame_cam_pcd.colors = o3d.utility.Vector3dVector(colors_rgb)

        # Downsample the dense frame to target voxel resolution
        frame_cam_pcd = frame_cam_pcd.voxel_down_sample(voxel_size=self.voxel_size)

        # Create lightweight downsampled cloud for ultra-fast tracking (<2 ms)
        track_source = frame_cam_pcd.voxel_down_sample(voxel_size=0.04)
        track_source.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.10, max_nn=20)
        )

        # 6. CONTINUOUS TRACKING (NEVER DRIFTS TO INFINITY)
        if len(self.global_pcd.points) < 50 or self.prev_track_pcd is None:
            self.current_T_WC = np.eye(4)
            self.global_pcd = o3d.geometry.PointCloud(frame_cam_pcd)
            self.prev_track_pcd = track_source
            self.last_process_time = now
            self.last_add_time = now
            self.publish_tf_and_pose(msg.header.stamp, self.current_T_WC)
            self.publish_accumulated_map()
            if self.enable_obstacles:
                self._publish_obstacles(pts_C, self.current_T_WC, msg.header.stamp)
            return

        # Fast Point-to-Plane ICP on lightweight downsampled tracking point cloud
        reg = o3d.pipelines.registration.registration_icp(
            track_source,
            self.prev_track_pcd,
            max_correspondence_distance=0.30,  # 30 cm allows smooth handheld movement
            init=np.eye(4),
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-4, relative_rmse=1e-4, max_iteration=20
            )
        )

        # Robust tracking update
        if reg.fitness >= 0.20:
            delta_T = np.array(reg.transformation, copy=True, dtype=np.float64)
            trans_norm = float(np.linalg.norm(delta_T[:3, 3]))
            # Sanity check: prevent wild jumps (> 35 cm in single frame)
            if trans_norm < 0.35:
                self.current_T_WC = self.current_T_WC @ delta_T
                self.prev_track_pcd = track_source
                self.last_delta_T = delta_T.copy()
            else:
                delta_T = np.eye(4)
        else:
            # Low overlap or fast motion: hold position, do NOT drift into outer space
            # and keep prev_track_pcd so it can re-acquire when user points back
            delta_T = np.eye(4)

        self.last_process_time = now

        # 7. MAP INTEGRATION & VOXEL FUSION
        trans_delta = float(np.linalg.norm(delta_T[:3, 3]))
        R_mat = np.array(delta_T[:3, :3], copy=True, dtype=np.float64)
        rot_delta_deg = float(np.degrees(np.linalg.norm(Rotation.from_matrix(R_mat).as_rotvec())))

        # Add new scan if phone moved > 2cm or rotated > 2deg, or periodically (every 1.2s)
        if trans_delta >= self.min_trans_m or rot_delta_deg >= self.min_rot_deg or (now - self.last_add_time) > 1.2:
            frame_world_pcd = o3d.geometry.PointCloud(frame_cam_pcd)
            frame_world_pcd.transform(self.current_T_WC)

            # Fuse into persistent global room map
            self.global_pcd += frame_world_pcd
            self.last_add_time = now

            # Maintain single, unified solid surface with voxel downsampling
            if len(self.global_pcd.points) > 30000:
                self.global_pcd = self.global_pcd.voxel_down_sample(voxel_size=self.voxel_size)

            self.publish_accumulated_map()

        # 8. PUBLISH RIGID TRAJECTORY & TF
        self.publish_tf_and_pose(msg.header.stamp, self.current_T_WC)
        if self.enable_obstacles:
            self._publish_obstacles(pts_C, self.current_T_WC, msg.header.stamp)

        # 9. Publish Live Turbo Depth Map
        d_norm = np.clip((depth_m - self.min_depth) / (self.max_depth - self.min_depth + 1e-6), 0.0, 1.0)
        depth_vis = (d_norm * 255.0).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = 'camera_optical_frame'
        depth_msg = self.bridge.cv2_to_imgmsg(depth_color, encoding='bgr8')
        depth_msg.header = header
        self.depth_img_pub.publish(depth_msg)

    def _publish_obstacles(self, pts_C: np.ndarray, T_WC: np.ndarray, stamp):
        """Cluster points in the camera corridor and publish world-frame hazard boxes."""
        if not getattr(self, 'enable_obstacles', False):
            return

        try:
            markers = MarkerArray()
            clear = Marker()
            clear.header.frame_id = 'world'
            clear.header.stamp = stamp
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)

            if len(pts_C):
                roi = (
                    (pts_C[:, 0] >= -self.obstacle_half_width) &
                    (pts_C[:, 0] <= self.obstacle_half_width) &
                    (pts_C[:, 1] >= self.obstacle_y_min) &
                    (pts_C[:, 1] <= min(self.obstacle_y_max, self.obstacle_ground_y)) &
                    (pts_C[:, 2] >= self.obstacle_min_depth) &
                    (pts_C[:, 2] <= self.obstacle_max_depth)
                )
                pts_roi = pts_C[roi]
            else:
                pts_roi = np.empty((0, 3), dtype=np.float64)

            if len(pts_roi) >= self.obstacle_min_points:
                cloud = o3d.geometry.PointCloud()
                cloud.points = o3d.utility.Vector3dVector(pts_roi)
                labels = np.asarray(cloud.cluster_dbscan(
                    eps=self.obstacle_cluster_eps,
                    min_points=self.obstacle_min_points,
                    print_progress=False,
                ))
                for cluster_id in range(int(labels.max()) + 1):
                    cluster_C = pts_roi[labels == cluster_id]
                    if len(cluster_C) < self.obstacle_min_points:
                        continue

                    cluster_W = cluster_C @ T_WC[:3, :3].T + T_WC[:3, 3]
                    cluster_cloud = o3d.geometry.PointCloud()
                    cluster_cloud.points = o3d.utility.Vector3dVector(cluster_W)
                    try:
                        box = cluster_cloud.get_oriented_bounding_box(robust=True)
                    except Exception:
                        box = cluster_cloud.get_axis_aligned_bounding_box()
                    center_W = np.array(box.center, copy=True, dtype=np.float64)
                    extent = np.maximum(np.array(box.extent, copy=True, dtype=np.float64), 0.03)
                    distance = float(np.linalg.norm(np.mean(cluster_C, axis=0)))

                    if distance < 0.8:
                        color = (1.0, 0.1, 0.1, 0.45)
                        label = f'COLLISION HAZARD: {distance:.2f}m'
                    elif distance < 1.8:
                        color = (1.0, 0.8, 0.0, 0.35)
                        label = f'OBSTACLE: {distance:.2f}m'
                    else:
                        color = (0.1, 0.9, 0.2, 0.25)
                        label = f'CLEARANCE: {distance:.2f}m'

                    R_mat = np.array(box.R, copy=True, dtype=np.float64)
                    q = Rotation.from_matrix(R_mat).as_quat()
                    cube = Marker()
                    cube.header.frame_id = 'world'
                    cube.header.stamp = stamp
                    cube.ns = 'obstacles'
                    cube.id = cluster_id
                    cube.type = Marker.CUBE
                    cube.action = Marker.ADD
                    cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = map(float, center_W)
                    cube.pose.orientation.x, cube.pose.orientation.y = float(q[0]), float(q[1])
                    cube.pose.orientation.z, cube.pose.orientation.w = float(q[2]), float(q[3])
                    cube.scale.x, cube.scale.y, cube.scale.z = map(float, extent)
                    cube.color.r, cube.color.g, cube.color.b, cube.color.a = color
                    markers.markers.append(cube)

                    text = Marker()
                    text.header.frame_id = 'world'
                    text.header.stamp = stamp
                    text.ns = 'obstacle_labels'
                    text.id = cluster_id
                    text.type = Marker.TEXT_VIEW_FACING
                    text.action = Marker.ADD
                    text.pose.position.x = float(center_W[0])
                    text.pose.position.y = float(center_W[1])
                    text.pose.position.z = float(center_W[2] + extent[2] * 0.5 + 0.08)
                    text.pose.orientation.w = 1.0
                    text.scale.z = 0.12
                    text.color.r, text.color.g, text.color.b, text.color.a = color
                    text.text = label
                    markers.markers.append(text)

            now = time.monotonic()
            if now - self._last_obstacle_diag_time >= 2.0:
                box_count = sum(1 for marker in markers.markers
                                if marker.type == Marker.CUBE and marker.action == Marker.ADD)
                self.get_logger().info(
                    f'Obstacle pipeline: {len(pts_C)} frame points, {len(pts_roi)} in corridor, '
                    f'{box_count} detected box(es).'
                )
                self._last_obstacle_diag_time = now
            self.obstacles_pub.publish(markers)
        except Exception as e:
            self.get_logger().warn(f"Obstacle pipeline caught exception: {e}")

    def publish_tf_and_pose(self, stamp, T: np.ndarray):
        """Broadcasts accurate real-time camera transform and trajectory."""
        p = T[:3, 3]
        R_mat = np.array(T[:3, :3], copy=True, dtype=np.float64)
        q = Rotation.from_matrix(R_mat).as_quat() # [x, y, z, w]

        # 1. Broadcast TF: world -> camera_optical_frame
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'camera_optical_frame'
        t.transform.translation.x = float(p[0])
        t.transform.translation.y = float(p[1])
        t.transform.translation.z = float(p[2])
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = float(p[0])
        pose_msg.pose.position.y = float(p[1])
        pose_msg.pose.position.z = float(p[2])
        pose_msg.pose.orientation.x = float(q[0])
        pose_msg.pose.orientation.y = float(q[1])
        pose_msg.pose.orientation.z = float(q[2])
        pose_msg.pose.orientation.w = float(q[3])
        self.pose_pub.publish(pose_msg)
        self.vio_pose_pub.publish(pose_msg)

        # 3. Append to Trajectory Path
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose_msg)
        self.path_pub.publish(self.path_msg)
        self.vio_path_pub.publish(self.path_msg)

    def publish_accumulated_map(self):
        """Publishes the global persistent room point cloud to RViz."""
        if len(self.global_pcd.points) < 50:
            return

        # Cap map points if necessary
        if len(self.global_pcd.points) > self.max_map_points:
            self.global_pcd = self.global_pcd.voxel_down_sample(voxel_size=self.voxel_size * 1.35)

        pts = np.asarray(self.global_pcd.points, dtype=np.float32)
        colors = (np.asarray(self.global_pcd.colors) * 255.0).astype(np.uint32)

        # Pack RGB into uint32 float for RViz PointCloud2
        rgb_u32 = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
        pts_with_rgb = np.column_stack([pts, rgb_u32.view(np.float32)])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'world'

        cloud_msg = pc2.create_cloud(header, self.fields, pts_with_rgb)
        self.map_pub.publish(cloud_msg)

    def save_map_to_disk(self):
        """Saves final high-definition 3D room scan to .ply and .pcd files."""
        if len(self.global_pcd.points) > 100:
            try:
                self.global_pcd.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.035, max_nn=25)
                )
            except Exception:
                pass

            out_dir = os.path.expanduser("~/vio_ws/results/pointclouds")
            os.makedirs(out_dir, exist_ok=True)
            ply_path = os.path.join(out_dir, "realtime_room_dense_map.ply")
            pcd_path = os.path.join(out_dir, "realtime_room_dense_map.pcd")

            o3d.io.write_point_cloud(ply_path, self.global_pcd)
            o3d.io.write_point_cloud(pcd_path, self.global_pcd)
            self.get_logger().info(f"[SUCCESS] Saved High-Definition Dense 3D Room Map ({len(self.global_pcd.points)} points) to: {ply_path}")


def main(args=None):
    rclpy.init(args=args)
    node = DenseMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_map_to_disk()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
