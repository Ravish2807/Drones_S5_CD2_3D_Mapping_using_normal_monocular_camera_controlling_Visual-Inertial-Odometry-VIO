import cv2
import numpy as np
from typing import Dict, List, Tuple, Optional, Any


class FeatureTrack:
    """Represents a tracked visual landmark across camera clones."""
    def __init__(self, track_id: int):
        self.track_id = track_id
        self.observations: Dict[int, Tuple[float, float, float, float]] = {} # clone_id -> (u, v, xn, yn)
        self.color: Tuple[int, int, int] = (200, 200, 200) # RGB
        self.is_lost: bool = False

    def add_observation(self, clone_id: int, uv: np.ndarray, norm_xy: np.ndarray):
        self.observations[clone_id] = (float(uv[0]), float(uv[1]), float(norm_xy[0]), float(norm_xy[1]))

    def set_color(self, rgb: Tuple[int, int, int]):
        self.color = rgb

    @property
    def length(self) -> int:
        return len(self.observations)


class FeatureTracker:
    """
    Image-based KLT Pyramidal optical flow tracker with FAST grid-bucketing.
    Operates strictly on 2D image arrays and camera intrinsic calibration.
    """

    def __init__(self, config: Dict[str, Any]):
        cam_cfg = config.get("camera", {})
        est_cfg = config.get("estimator", {})

        self.width = cam_cfg.get("width", 640)
        self.height = cam_cfg.get("height", 480)
        self.max_features = est_cfg.get("max_features", 150)
        self.min_features = est_cfg.get("min_features", 50)
        self.min_dist = est_cfg.get("min_feature_dist", 15.0)

        self.grid_rows = est_cfg.get("grid_rows", 4)
        self.grid_cols = est_cfg.get("grid_cols", 5)

        # Camera intrinsics
        intrinsics = cam_cfg.get("intrinsics", {})
        self.fx = float(intrinsics.get("fx", 400.0))
        self.fy = float(intrinsics.get("fy", 400.0))
        self.cx = float(intrinsics.get("cx", 320.0))
        self.cy = float(intrinsics.get("cy", 240.0))
        self.K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float64)
        self.K_inv = np.linalg.inv(self.K)

        self.prev_img: Optional[np.ndarray] = None
        self.prev_pts: Optional[np.ndarray] = None  # shape (N, 2)
        self.prev_ids: Optional[np.ndarray] = None  # shape (N,)
        
        self.next_feature_id = 0
        self.active_tracks: Dict[int, FeatureTrack] = {}

    def _pixel_to_normalized(self, pts: np.ndarray) -> np.ndarray:
        """Converts pixel coordinates (u, v) to normalized coordinates (xn, yn)."""
        pts_homo = np.hstack([pts, np.ones((len(pts), 1))])
        norm_homo = (self.K_inv @ pts_homo.T).T
        return norm_homo[:, :2]

    def track_frame(self, img: np.ndarray, clone_id: int) -> Tuple[List[FeatureTrack], List[FeatureTrack]]:
        """
        Processes incoming camera image:
        1. Tracks existing features from previous frame via LK optical flow + FB check.
        2. Detects new features in under-populated grid cells.
        3. Returns (active_tracks, lost_tracks_ready_for_update).
        """
        if len(img.shape) == 3:
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        else:
            img_gray = img
            img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        lost_tracks = []

        if self.prev_img is None or self.prev_pts is None or len(self.prev_pts) == 0:
            new_pts = self._detect_features_grid(img_gray, num_to_detect=self.max_features)
            if len(new_pts) > 0:
                new_ids = np.arange(self.next_feature_id, self.next_feature_id + len(new_pts))
                self.next_feature_id += len(new_pts)

                norm_pts = self._pixel_to_normalized(new_pts)
                for pt_id, pt, norm_pt in zip(new_ids, new_pts, norm_pts):
                    track = FeatureTrack(int(pt_id))
                    u, v = int(np.clip(round(pt[0]), 0, self.width - 1)), int(np.clip(round(pt[1]), 0, self.height - 1))
                    track.set_color(tuple(int(c) for c in img_rgb[v, u]))
                    track.add_observation(clone_id, pt, norm_pt)
                    self.active_tracks[int(pt_id)] = track

                self.prev_pts = new_pts.astype(np.float32)
                self.prev_ids = new_ids
            self.prev_img = img_gray
            return list(self.active_tracks.values()), []

        # Pyramidal LK optical flow
        curr_pts, status_fwd, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_img, img_gray, self.prev_pts, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        # Forward-Backward consistency check
        prev_pts_back, status_bwd, _ = cv2.calcOpticalFlowPyrLK(
            img_gray, self.prev_img, curr_pts, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        fb_err = np.linalg.norm(self.prev_pts - prev_pts_back, axis=1)
        valid_mask = (status_fwd.ravel() == 1) & (status_bwd.ravel() == 1) & (fb_err < 1.0)

        # Boundary check
        in_bounds = (
            (curr_pts[:, 0] >= 10) & (curr_pts[:, 0] < self.width - 10) &
            (curr_pts[:, 1] >= 10) & (curr_pts[:, 1] < self.height - 10)
        )
        valid_mask = valid_mask & in_bounds

        surviving_pts = curr_pts[valid_mask]
        surviving_ids = self.prev_ids[valid_mask]
        lost_ids = self.prev_ids[~valid_mask]

        for lid in lost_ids:
            if int(lid) in self.active_tracks:
                t = self.active_tracks.pop(int(lid))
                t.is_lost = True
                lost_tracks.append(t)

        if len(surviving_pts) > 0:
            norm_surviving = self._pixel_to_normalized(surviving_pts)
            for pt_id, pt, norm_pt in zip(surviving_ids, surviving_pts, norm_surviving):
                if int(pt_id) in self.active_tracks:
                    t = self.active_tracks[int(pt_id)]
                    u, v = int(np.clip(round(pt[0]), 0, self.width - 1)), int(np.clip(round(pt[1]), 0, self.height - 1))
                    t.set_color(tuple(int(c) for c in img_rgb[v, u]))
                    t.add_observation(clone_id, pt, norm_pt)

        # Grid bucketing replenishment
        num_needed = self.max_features - len(surviving_pts)
        if num_needed > 10:
            new_pts = self._detect_features_grid(img_gray, num_to_detect=num_needed, existing_pts=surviving_pts)
            if len(new_pts) > 0:
                new_ids = np.arange(self.next_feature_id, self.next_feature_id + len(new_pts))
                self.next_feature_id += len(new_pts)

                norm_new = self._pixel_to_normalized(new_pts)
                for pt_id, pt, norm_pt in zip(new_ids, new_pts, norm_new):
                    track = FeatureTrack(int(pt_id))
                    u, v = int(np.clip(round(pt[0]), 0, self.width - 1)), int(np.clip(round(pt[1]), 0, self.height - 1))
                    track.set_color(tuple(int(c) for c in img_rgb[v, u]))
                    track.add_observation(clone_id, pt, norm_pt)
                    self.active_tracks[int(pt_id)] = track

                surviving_pts = np.vstack([surviving_pts, new_pts]) if len(surviving_pts) > 0 else new_pts
                surviving_ids = np.concatenate([surviving_ids, new_ids]) if len(surviving_ids) > 0 else new_ids

        self.prev_img = img_gray
        self.prev_pts = surviving_pts.astype(np.float32)
        self.prev_ids = surviving_ids

        return list(self.active_tracks.values()), lost_tracks

    def _detect_features_grid(self, img_gray: np.ndarray, num_to_detect: int,
                              existing_pts: Optional[np.ndarray] = None) -> np.ndarray:
        """Detects features uniformly across a spatial grid."""
        mask = np.full((self.height, self.width), 255, dtype=np.uint8)
        if existing_pts is not None and len(existing_pts) > 0:
            for pt in existing_pts:
                cv2.circle(mask, (int(round(pt[0])), int(round(pt[1]))), int(self.min_dist), 0, -1)

        detected = cv2.goodFeaturesToTrack(
            img_gray, maxCorners=num_to_detect, qualityLevel=0.01,
            minDistance=self.min_dist, mask=mask, blockSize=3, useHarrisDetector=False
        )

        if detected is None:
            return np.empty((0, 2), dtype=np.float32)
        return detected.reshape(-1, 2)
