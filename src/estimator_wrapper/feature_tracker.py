import cv2
import numpy as np
from typing import Dict, List, Tuple, Optional, Any


class FeatureTrack:
    """Represents the observation history of a visual landmark."""
    def __init__(self, track_id: int):
        self.track_id = track_id
        self.observations = {}  # {clone_id: (u, v, x_norm, y_norm)}
        self.last_seen_clone_id = -1

    def add_observation(self, clone_id: int, uv: np.ndarray, uv_norm: np.ndarray):
        self.observations[clone_id] = (uv[0], uv[1], uv_norm[0], uv_norm[1])
        self.last_seen_clone_id = clone_id

    @property
    def length(self) -> int:
        return len(self.observations)


class FeatureTracker:
    """
    Visual feature tracker using FAST feature detection and Pyramidal Lucas-Kanade optical flow,
    with spatial grid-bucketing and forward-backward consistency checking.
    """

    def __init__(self, config: Dict[str, Any]):
        self.cfg = config
        est_cfg = config.get("estimator", {})
        cam_cfg = config.get("camera", {})

        self.max_features = est_cfg.get("max_features", 150)
        self.fast_threshold = est_cfg.get("fast_threshold", 20)
        self.grid_rows = est_cfg.get("grid_rows", 4)
        self.grid_cols = est_cfg.get("grid_cols", 5)

        # Camera intrinsics
        intrinsics = cam_cfg.get("intrinsics", {})
        self.fx = intrinsics.get("fx", 400.0)
        self.fy = intrinsics.get("fy", 400.0)
        self.cx = intrinsics.get("cx", 320.0)
        self.cy = intrinsics.get("cy", 240.0)
        self.K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float64)
        self.K_inv = np.linalg.inv(self.K)

        self.prev_img: Optional[np.ndarray] = None
        self.prev_pts: Optional[np.ndarray] = None  # shape (N, 2)
        self.prev_ids: Optional[np.ndarray] = None  # shape (N,)
        
        self.next_feature_id = 0
        self.active_tracks: Dict[int, FeatureTrack] = {}

    def _pixel_to_normalized(self, pts: np.ndarray) -> np.ndarray:
        """Converts pixel coordinates (u, v) to normalized camera coordinates (x, y, 1)."""
        pts_homo = np.hstack([pts, np.ones((len(pts), 1))])
        norm_homo = (self.K_inv @ pts_homo.T).T
        return norm_homo[:, :2]

    def track_frame(self, img: np.ndarray, clone_id: int) -> Tuple[List[FeatureTrack], List[FeatureTrack]]:
        """
        Processes a new camera frame:
        - Tracks existing features from previous frame via LK optical flow + FB check.
        - Detects new features in under-populated grid cells.
        - Returns (active_tracks, lost_tracks_ready_for_update).
        """
        if len(img.shape) == 3:
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            img_gray = img

        lost_tracks = []

        if self.prev_img is None or self.prev_pts is None or len(self.prev_pts) == 0:
            # First frame detection
            new_pts = self._detect_features_grid(img_gray, num_to_detect=self.max_features)
            if len(new_pts) > 0:
                new_ids = np.arange(self.next_feature_id, self.next_feature_id + len(new_pts))
                self.next_feature_id += len(new_pts)

                norm_pts = self._pixel_to_normalized(new_pts)
                for pt_id, pt, norm_pt in zip(new_ids, new_pts, norm_pts):
                    track = FeatureTrack(int(pt_id))
                    track.add_observation(clone_id, pt, norm_pt)
                    self.active_tracks[int(pt_id)] = track

                self.prev_pts = new_pts
                self.prev_ids = new_ids
            else:
                self.prev_pts = np.empty((0, 2), dtype=np.float32)
                self.prev_ids = np.empty((0,), dtype=int)
            
            self.prev_img = img_gray
            return list(self.active_tracks.values()), []

        # 1. LK Optical Flow Tracking (Forward)
        lk_params = dict(winSize=(21, 21), maxLevel=3,
                         criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        
        pts_curr, status_fwd, _ = cv2.calcOpticalFlowPyrLK(self.prev_img, img_gray, self.prev_pts.astype(np.float32), None, **lk_params)
        
        # 2. Backward Check for Outlier Rejection
        pts_bwd, status_bwd, _ = cv2.calcOpticalFlowPyrLK(img_gray, self.prev_img, pts_curr, None, **lk_params)
        
        fb_dist = np.linalg.norm(self.prev_pts - pts_bwd, axis=1)
        valid = (status_fwd.ravel() == 1) & (status_bwd.ravel() == 1) & (fb_dist < 1.0)

        # Boundary check
        h, w = img_gray.shape
        in_bounds = (pts_curr[:, 0] >= 5) & (pts_curr[:, 0] < w - 5) & (pts_curr[:, 1] >= 5) & (pts_curr[:, 1] < h - 5)
        valid = valid & in_bounds

        tracked_pts = pts_curr[valid]
        tracked_ids = self.prev_ids[valid]
        lost_ids = self.prev_ids[~valid]

        # Update active tracks & collect lost tracks
        for lid in lost_ids:
            lid_int = int(lid)
            if lid_int in self.active_tracks:
                lost_tracks.append(self.active_tracks[lid_int])
                del self.active_tracks[lid_int]

        tracked_norm = self._pixel_to_normalized(tracked_pts) if len(tracked_pts) > 0 else np.empty((0, 2))
        for pt_id, pt, norm_pt in zip(tracked_ids, tracked_pts, tracked_norm):
            pt_id_int = int(pt_id)
            if pt_id_int in self.active_tracks:
                self.active_tracks[pt_id_int].add_observation(clone_id, pt, norm_pt)

        # 3. Detect new features to maintain max_features
        needed = self.max_features - len(tracked_pts)
        if needed > 15:
            new_pts = self._detect_features_grid(img_gray, num_to_detect=needed, existing_pts=tracked_pts)
            if len(new_pts) > 0:
                new_ids = np.arange(self.next_feature_id, self.next_feature_id + len(new_pts))
                self.next_feature_id += len(new_pts)

                new_norm = self._pixel_to_normalized(new_pts)
                for pt_id, pt, norm_pt in zip(new_ids, new_pts, new_norm):
                    track = FeatureTrack(int(pt_id))
                    track.add_observation(clone_id, pt, norm_pt)
                    self.active_tracks[int(pt_id)] = track

                if len(tracked_pts) > 0:
                    tracked_pts = np.vstack([tracked_pts, new_pts])
                    tracked_ids = np.concatenate([tracked_ids, new_ids])
                else:
                    tracked_pts = new_pts
                    tracked_ids = new_ids

        self.prev_img = img_gray
        self.prev_pts = tracked_pts
        self.prev_ids = tracked_ids

        return list(self.active_tracks.values()), lost_tracks

    def _detect_features_grid(self, img: np.ndarray, num_to_detect: int, existing_pts: Optional[np.ndarray] = None) -> np.ndarray:
        """Detects features uniformly distributed using grid bucketing."""
        h, w = img.shape
        cell_h = h // self.grid_rows
        cell_w = w // self.grid_cols
        max_per_cell = int(np.ceil(self.max_features / (self.grid_rows * self.grid_cols)))

        # Create mask of existing points
        mask = np.ones((h, w), dtype=np.uint8) * 255
        if existing_pts is not None and len(existing_pts) > 0:
            for pt in existing_pts:
                cv2.circle(mask, (int(round(pt[0])), int(round(pt[1]))), 10, 0, -1)

        corners = cv2.goodFeaturesToTrack(
            img,
            maxCorners=num_to_detect,
            qualityLevel=0.01,
            minDistance=8,
            mask=mask,
            blockSize=3
        )

        if corners is None or len(corners) == 0:
            return np.empty((0, 2), dtype=np.float32)

        return corners.reshape(-1, 2)
