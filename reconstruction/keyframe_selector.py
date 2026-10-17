#!/usr/bin/env python3
"""
Keyframe Selection Engine for Decoupled VIO-Guided 3D Reconstruction.
Determines if an incoming camera frame should be promoted to a Keyframe based on:
  1. Translation Baseline: ||p_k - p_last|| > d_min
  2. Geodesic Rotation:    d_R(R_k, R_last) > theta_min
  3. Feature Visual Overlap: overlap(F_k, F_last) < tau
"""

import numpy as np
from typing import Optional, Tuple, Dict, Any, List
from .keyframe_bundle import Keyframe, CameraIntrinsics


class KeyframeSelector:
    """
    Evaluates candidate camera frames against the most recent keyframe
    to filter redundant views and select optimal baselines for multi-view stereo.
    """

    def __init__(
        self,
        min_translation_m: float = 0.15,
        min_rotation_deg: float = 8.0,
        min_feature_overlap: float = 0.65,
        max_time_interval_sec: float = 3.0,
        min_tracked_features: int = 15
    ):
        self.min_translation_m = float(min_translation_m)
        self.min_rotation_deg = float(min_rotation_deg)
        self.min_feature_overlap = float(min_feature_overlap)
        self.max_time_interval_sec = float(max_time_interval_sec)
        self.min_tracked_features = int(min_tracked_features)

        self.last_keyframe: Optional[Keyframe] = None
        self.keyframe_count: int = 0
        self.total_frames_evaluated: int = 0

    def evaluate(
        self,
        timestamp: float,
        image: np.ndarray,
        T_WC: np.ndarray,
        intrinsics: CameraIntrinsics,
        feature_ids: Optional[List[int]] = None,
        feature_points: Optional[np.ndarray] = None,
        tracking_quality: Optional[Dict[str, Any]] = None
    ) -> Tuple[bool, str, Optional[Keyframe]]:
        """
        Evaluates candidate frame and returns (is_keyframe, reason_str, keyframe_obj).
        """
        self.total_frames_evaluated += 1
        tracking_quality = tracking_quality or {}

        # 1. First frame is always a Keyframe
        if self.last_keyframe is None:
            kf = Keyframe(
                frame_id=self.keyframe_count,
                timestamp=timestamp,
                image=image.copy(),
                T_WC=T_WC.copy(),
                intrinsics=intrinsics,
                tracking_quality=tracking_quality,
                feature_ids=list(feature_ids) if feature_ids is not None else None,
                feature_points=feature_points.copy() if feature_points is not None else None
            )
            self.last_keyframe = kf
            self.keyframe_count += 1
            return True, "initial_frame", kf

        # 2. Extract current and last pose components
        p_curr = T_WC[0:3, 3]
        R_curr = T_WC[0:3, 0:3]
        p_last = self.last_keyframe.p_WC
        R_last = self.last_keyframe.R_WC

        # 3. Translation metric: ||p_curr - p_last||
        dist_m = float(np.linalg.norm(p_curr - p_last))

        # 4. Geodesic rotation metric on SO(3): arccos((Tr(R_last^T * R_curr) - 1) / 2)
        R_rel = R_last.T @ R_curr
        trace_val = np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0)
        angle_rad = np.arccos(trace_val)
        angle_deg = float(np.degrees(angle_rad))

        # 5. Visual feature overlap ratio
        overlap_ratio = 1.0
        if feature_ids is not None and self.last_keyframe.feature_ids is not None:
            curr_set = set(feature_ids)
            last_set = set(self.last_keyframe.feature_ids)
            if len(last_set) > 0:
                overlap_ratio = float(len(curr_set.intersection(last_set)) / len(last_set))

        # 6. Time interval check
        time_elapsed = float(timestamp - self.last_keyframe.timestamp)

        # Decision Logic:
        is_kf = False
        reason = ""

        if dist_m >= self.min_translation_m:
            is_kf = True
            reason = f"translation_exceeded ({dist_m:.3f}m >= {self.min_translation_m:.3f}m)"
        elif angle_deg >= self.min_rotation_deg:
            is_kf = True
            reason = f"rotation_exceeded ({angle_deg:.2f}° >= {self.min_rotation_deg:.2f}°)"
        elif overlap_ratio <= self.min_feature_overlap:
            is_kf = True
            reason = f"feature_overlap_dropped ({overlap_ratio:.2f} <= {self.min_feature_overlap:.2f})"
        elif time_elapsed >= self.max_time_interval_sec:
            is_kf = True
            reason = f"max_time_interval ({time_elapsed:.2f}s >= {self.max_time_interval_sec:.2f}s)"

        if is_kf:
            kf = Keyframe(
                frame_id=self.keyframe_count,
                timestamp=timestamp,
                image=image.copy(),
                T_WC=T_WC.copy(),
                intrinsics=intrinsics,
                tracking_quality={
                    **tracking_quality,
                    "trigger_reason": reason,
                    "delta_trans_m": dist_m,
                    "delta_rot_deg": angle_deg,
                    "feature_overlap": overlap_ratio
                },
                feature_ids=list(feature_ids) if feature_ids is not None else None,
                feature_points=feature_points.copy() if feature_points is not None else None
            )
            self.last_keyframe = kf
            self.keyframe_count += 1
            return True, reason, kf

        return False, "redundant_pose", None

    def reset(self):
        self.last_keyframe = None
        self.keyframe_count = 0
        self.total_frames_evaluated = 0
