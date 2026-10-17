#!/usr/bin/env python3
"""
Asynchronous Decoupled Reconstruction Worker.
Bridges the fast real-time VIO Kalman loop (20-30 Hz) with the heavier
multi-view dense stereo and 3D surface reconstruction pipeline.
Ensures VIO state propagation is never blocked by dense 3D processing.
"""

import os
import time
import queue
import threading
import numpy as np
import open3d as o3d
from typing import Optional, List, Dict, Any, Tuple

from .keyframe_bundle import Keyframe, CameraIntrinsics, KeyframeDatabase
from .keyframe_selector import KeyframeSelector
from .mvs_depth_estimator import MVSDepthEstimator
from .depth_fusion import DepthFusion
from .surface_mesher import SurfaceMesher


class ReconstructionWorker:
    """
    Decoupled multi-threaded worker for real-time VIO-guided 3D reconstruction.
    """

    def __init__(
        self,
        config: Optional[Dict[str, Any]] = None,
        output_dir: str = "results"
    ):
        self.config = config or {}
        self.output_dir = output_dir

        self.kf_dir = os.path.join(output_dir, "keyframes")
        self.pcd_dir = os.path.join(output_dir, "pointclouds")
        self.depth_dir = os.path.join(output_dir, "depth_maps")

        os.makedirs(self.kf_dir, exist_ok=True)
        os.makedirs(self.pcd_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)

        # Core Components
        self.database = KeyframeDatabase()
        self.selector = KeyframeSelector(
            min_translation_m=self.config.get("min_translation_m", 0.15),
            min_rotation_deg=self.config.get("min_rotation_deg", 8.0),
            min_feature_overlap=self.config.get("min_feature_overlap", 0.65)
        )
        self.depth_estimator = MVSDepthEstimator(
            min_depth=self.config.get("min_depth", 0.5),
            max_depth=self.config.get("max_depth", 8.0),
            num_depth_planes=self.config.get("num_depth_planes", 48)
        )
        self.fusion = DepthFusion(
            voxel_size_m=self.config.get("voxel_size_m", 0.03),
            min_depth=self.config.get("min_depth", 0.5),
            max_depth=self.config.get("max_depth", 8.0)
        )
        self.mesher = SurfaceMesher(
            poisson_depth=self.config.get("poisson_depth", 8)
        )

        # Asynchronous Threading
        self.kf_queue: queue.Queue = queue.Queue(maxsize=100)
        self.is_running: bool = False
        self.worker_thread: Optional[threading.Thread] = None

        # Reconstructed 3D Geometry Cache
        self.latest_pcd: Optional[o3d.geometry.PointCloud] = None
        self.latest_mesh: Optional[o3d.geometry.TriangleMesh] = None
        self.lock = threading.Lock()

    def start(self):
        """Starts asynchronous background reconstruction worker thread."""
        if self.is_running:
            return
        self.is_running = True
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True, name="ReconstructionWorker")
        self.worker_thread.start()
        print("[ReconstructionWorker] Background worker thread started successfully.")

    def stop(self, wait_completion: bool = True):
        """Signals worker to finish remaining keyframes and cleanly terminate."""
        if not self.is_running:
            return
        print("[ReconstructionWorker] Stopping worker and finalizing reconstruction...")
        self.is_running = False
        if wait_completion and self.worker_thread is not None:
            self.worker_thread.join(timeout=15.0)
        self.finalize()
        print("[ReconstructionWorker] Stopped.")

    def push_frame(
        self,
        timestamp: float,
        image: np.ndarray,
        T_WC: np.ndarray,
        intrinsics: CameraIntrinsics,
        feature_ids: Optional[List[int]] = None,
        feature_points: Optional[np.ndarray] = None,
        tracking_quality: Optional[Dict[str, Any]] = None
    ) -> bool:
        """
        Fast non-blocking call executed in VIO loop.
        Evaluates frame for keyframe eligibility and queues if accepted.
        """
        is_kf, reason, kf = self.selector.evaluate(
            timestamp=timestamp,
            image=image,
            T_WC=T_WC,
            intrinsics=intrinsics,
            feature_ids=feature_ids,
            feature_points=feature_points,
            tracking_quality=tracking_quality
        )

        if is_kf and kf is not None:
            try:
                self.kf_queue.put_nowait(kf)
                return True
            except queue.Full:
                print("[ReconstructionWorker] Warning: Keyframe queue full, dropping candidate.")
                return False

        return False

    def _worker_loop(self):
        """Asynchronous processing loop."""
        while self.is_running or not self.kf_queue.empty():
            try:
                kf = self.kf_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            # Add to database
            self.database.add(kf)
            kf_idx = len(self.database) - 1
            print(f"[ReconstructionWorker] Processing Keyframe #{kf.frame_id} (Total in DB: {len(self.database)})...")

            # Need at least 2 keyframes to estimate multi-view depth
            if len(self.database) >= 2:
                self.depth_estimator.process_keyframe(kf_idx, self.database.keyframes)

                # Recompute depth for previous keyframe if it had insufficient neighbors previously
                if kf_idx == 1 and self.database[0].depth_map is None:
                    self.depth_estimator.process_keyframe(0, self.database.keyframes)

            self.kf_queue.task_done()

    def finalize(self):
        """Runs multi-view depth fusion and surface meshing on all gathered keyframes."""
        if len(self.database) < 2:
            print("[ReconstructionWorker] Not enough keyframes (< 2) for 3D reconstruction.")
            return

        print(f"[ReconstructionWorker] Finalizing 3D reconstruction from {len(self.database)} keyframes...")

        # 1. Save Keyframe database to disk
        self.database.save_to_disk(self.kf_dir)

        # 2. Fuse Depth Maps into Dense Point Cloud
        dense_pcd = self.fusion.fuse_point_clouds(self.database.keyframes)
        pcd_out = os.path.join(self.pcd_dir, "dense_reconstruction.ply")
        self.fusion.export_point_cloud(dense_pcd, pcd_out)

        # 3. Surface Reconstruction (Poisson Mesh)
        if len(dense_pcd.points) > 50:
            mesh = self.mesher.reconstruct_poisson_mesh(dense_pcd)
            mesh_out = os.path.join(self.pcd_dir, "dense_reconstruction_mesh.ply")
            self.mesher.export_mesh(mesh, mesh_out)

            # Generate 3D Obstacle Voxel Grid
            vox_out = os.path.join(self.pcd_dir, "dense_reconstruction_voxels.ply")
            self.mesher.export_voxel_grid(dense_pcd, vox_out, voxel_size=0.10)

            with self.lock:
                self.latest_pcd = dense_pcd
                self.latest_mesh = mesh
        else:
            print("[ReconstructionWorker] Insufficient points for surface meshing.")
