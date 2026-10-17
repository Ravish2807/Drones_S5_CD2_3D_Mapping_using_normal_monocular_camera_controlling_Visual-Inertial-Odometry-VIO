#!/usr/bin/env python3
"""
Multi-View Stereo (MVS) Depth Estimation Engine.
Takes keyframe images and calibrated VIO camera optical poses (T_WC) to generate
dense per-view metric depth maps and photometric confidence maps.
Supports Plane-Sweep Homography MVS, Epipolar Semi-Global Matching, and deep MVS hooks.
"""

import os
import cv2
import numpy as np
from typing import List, Tuple, Dict, Any, Optional
from .keyframe_bundle import Keyframe, CameraIntrinsics


class MVSDepthEstimator:
    """
    Estimates dense metric depth maps from multi-view keyframes and VIO poses T_WC.
    """

    def __init__(
        self,
        min_depth: float = 0.5,
        max_depth: float = 8.0,
        num_depth_planes: int = 64,
        patch_size: int = 5,
        min_baseline_angle_deg: float = 3.0,
        max_baseline_angle_deg: float = 45.0,
        confidence_threshold: float = 0.40,
        method: str = "plane_sweep" # "plane_sweep", "sgbm", or "hybrid"
    ):
        self.min_depth = float(min_depth)
        self.max_depth = float(max_depth)
        self.num_depth_planes = int(num_depth_planes)
        self.patch_size = int(patch_size)
        self.min_baseline_angle_deg = float(min_baseline_angle_deg)
        self.max_baseline_angle_deg = float(max_baseline_angle_deg)
        self.confidence_threshold = float(confidence_threshold)
        self.method = method

    def select_neighbor_keyframes(
        self,
        ref_idx: int,
        keyframes: List[Keyframe],
        max_neighbors: int = 4
    ) -> List[int]:
        """
        Selects optimal source keyframes for multi-view stereo matching
        based on baseline distance, viewing angle baseline, and visual overlap.
        """
        ref_kf = keyframes[ref_idx]
        ref_p = ref_kf.p_WC
        ref_v = ref_kf.viewing_direction

        candidates = []
        for j, src_kf in enumerate(keyframes):
            if j == ref_idx:
                continue

            src_p = src_kf.p_WC
            src_v = src_kf.viewing_direction

            # Baseline vector
            b_vec = src_p - ref_p
            baseline = float(np.linalg.norm(b_vec))
            if baseline < 0.05: # Reject almost identical viewpoints
                continue

            # Viewing angle between optical axes
            dot_v = np.clip(np.dot(ref_v, src_v), -1.0, 1.0)
            angle_deg = float(np.degrees(np.arccos(dot_v)))

            if self.min_baseline_angle_deg <= angle_deg <= self.max_baseline_angle_deg:
                # Score combines adequate baseline and similar viewing angle
                score = baseline / (1.0 + np.abs(angle_deg - 15.0) * 0.1)
                candidates.append((score, j))

        # Fallback if angle filtering was too strict: pick closest temporal/spatial neighbors
        if len(candidates) < 2:
            candidates = []
            for j, src_kf in enumerate(keyframes):
                if j == ref_idx:
                    continue
                dist = np.linalg.norm(src_kf.p_WC - ref_p)
                if dist > 0.05:
                    candidates.append((-dist, j))

        candidates.sort(key=lambda x: x[0], reverse=True)
        return [c[1] for c in candidates[:max_neighbors]]

    def estimate_depth_plane_sweep(
        self,
        ref_kf: Keyframe,
        src_keyframes: List[Keyframe]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes dense depth map via Plane-Sweep Multi-View Stereo.
        Warp source images to reference frame at discrete depth hypotheses d_k,
        accumulate photometric matching costs, and find argmin depth.
        """
        ref_img = ref_kf.image
        if len(ref_img.shape) == 3:
            ref_gray = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)
        else:
            ref_gray = ref_img.copy()

        H, W = ref_gray.shape
        K = ref_kf.intrinsics.K
        K_inv = np.linalg.inv(K)

        # Scale down for faster plane-sweep computation, then upsample
        scale = 0.5
        h_s, w_s = int(H * scale), int(W * scale)
        K_s = K.copy()
        K_s[0, 0] *= scale
        K_s[1, 1] *= scale
        K_s[0, 2] *= scale
        K_s[1, 2] *= scale
        K_s_inv = np.linalg.inv(K_s)

        ref_small = cv2.resize(ref_gray, (w_s, h_s), interpolation=cv2.INTER_AREA).astype(np.float32)

        # Discretize inverse depth (1/d) linearly for uniform pixel disparity spacing
        inv_depths = np.linspace(1.0 / self.max_depth, 1.0 / self.min_depth, self.num_depth_planes)
        depth_candidates = 1.0 / inv_depths

        # Cost volume: (num_depths, h_s, w_s)
        cost_volume = np.zeros((self.num_depth_planes, h_s, w_s), dtype=np.float32)

        # Principal plane normal in reference camera coordinates
        n_ref = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        T_WC_ref = ref_kf.T_WC
        R_WC_ref = T_WC_ref[0:3, 0:3]
        p_WC_ref = T_WC_ref[0:3, 3]

        valid_sources = 0

        for src_kf in src_keyframes:
            src_img = src_kf.image
            if len(src_img.shape) == 3:
                src_gray = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
            else:
                src_gray = src_img
            src_small = cv2.resize(src_gray, (w_s, h_s), interpolation=cv2.INTER_AREA).astype(np.float32)

            # Relative pose: T_src_ref = T_WC_src^(-1) * T_WC_ref
            T_WC_src = src_kf.T_WC
            R_WC_src = T_WC_src[0:3, 0:3]
            p_WC_src = T_WC_src[0:3, 3]

            R_src_ref = R_WC_src.T @ R_WC_ref
            t_src_ref = R_WC_src.T @ (p_WC_ref - p_WC_src)

            # Check for non-zero baseline
            if np.linalg.norm(t_src_ref) < 1e-4:
                continue

            valid_sources += 1

            for d_idx, d in enumerate(depth_candidates):
                # Plane homography: H(d) = K_s * (R + t * n^T / d) * K_s_inv
                H_d = K_s @ (R_src_ref + np.outer(t_src_ref, n_ref) / d) @ K_s_inv

                # Warp source view to reference view
                warped_src = cv2.warpPerspective(
                    src_small,
                    H_d,
                    (w_s, h_s),
                    flags=cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                    borderValue=0
                )

                # Matching cost: Absolute Difference with patch aggregation
                diff = np.abs(ref_small - warped_src)
                mask = (warped_src > 5) & (ref_small > 5)
                diff[~mask] = 255.0

                # Spatial patch aggregation
                blurred_diff = cv2.boxFilter(diff, ddepth=-1, ksize=(self.patch_size, self.patch_size))
                cost_volume[d_idx] += blurred_diff

        if valid_sources > 0:
            cost_volume /= valid_sources
        else:
            return np.full((H, W), (self.min_depth + self.max_depth) * 0.5, dtype=np.float32), np.zeros((H, W), dtype=np.float32)

        # Winner-Takes-All depth selection
        min_cost_indices = np.argmin(cost_volume, axis=0)
        min_costs = np.min(cost_volume, axis=0)
        mean_costs = np.mean(cost_volume, axis=0)

        # Texture gradient check: reject flat background / textureless pixels
        grad_x = cv2.Sobel(ref_small, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(ref_small, cv2.CV_32F, 0, 1, ksize=3)
        texture_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        has_texture = texture_magnitude > 2.0

        # Confidence = cost margin * texture presence
        cost_margin = np.clip((mean_costs - min_costs) / (mean_costs + 1e-5), 0.0, 1.0)
        confidence_small = np.where(has_texture, cost_margin, 0.0).astype(np.float32)
        depth_small = depth_candidates[min_cost_indices].astype(np.float32)

        # Upsample back to original resolution with bilateral guided filter
        depth_map = cv2.resize(depth_small, (W, H), interpolation=cv2.INTER_CUBIC)
        confidence_map = cv2.resize(confidence_small, (W, H), interpolation=cv2.INTER_LINEAR)

        # Median filter to eliminate speckle noise
        depth_map = cv2.medianBlur(depth_map, 5)

        return depth_map, confidence_map

    def estimate_depth_sgbm(
        self,
        ref_kf: Keyframe,
        src_kf: Keyframe
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Fast Epipolar Semi-Global Matching (SGBM) between two calibrated keyframes.
        """
        ref_gray = cv2.cvtColor(ref_kf.image, cv2.COLOR_BGR2GRAY) if len(ref_kf.image.shape) == 3 else ref_kf.image
        src_gray = cv2.cvtColor(src_kf.image, cv2.COLOR_BGR2GRAY) if len(src_kf.image.shape) == 3 else src_kf.image

        H, W = ref_gray.shape
        K = ref_kf.intrinsics.K

        # Epipolar rectification
        T_rel = np.linalg.inv(src_kf.T_WC) @ ref_kf.T_WC
        R = T_rel[0:3, 0:3]
        t = T_rel[0:3, 3]

        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K, None, K, None, (W, H), R, t, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
        )

        map1x, map1y = cv2.initUndistortRectifyMap(K, None, R1, P1, (W, H), cv2.CV_32FC1)
        map2x, map2y = cv2.initUndistortRectifyMap(K, None, R2, P2, (W, H), cv2.CV_32FC1)

        rect_ref = cv2.remap(ref_gray, map1x, map1y, cv2.INTER_LINEAR)
        rect_src = cv2.remap(src_gray, map2x, map2y, cv2.INTER_LINEAR)

        num_disp = 16 * 6
        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disp,
            blockSize=5,
            P1=8 * 1 * 25,
            P2=32 * 1 * 25,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )

        disp = stereo.compute(rect_ref, rect_src).astype(np.float32) / 16.0
        disp[disp <= 0] = 0.1

        baseline = float(np.linalg.norm(t))
        fx = K[0, 0]
        depth_rect = (fx * baseline) / disp
        depth_rect = np.clip(depth_rect, self.min_depth, self.max_depth)

        # Un-rectify depth back to reference image coordinates
        inv_map1x, inv_map1y = cv2.initUndistortRectifyMap(P1[0:3, 0:3], None, R1.T, K, (W, H), cv2.CV_32FC1)
        depth_map = cv2.remap(depth_rect, inv_map1x, inv_map1y, cv2.INTER_LINEAR)

        confidence_map = (disp > 1.0).astype(np.float32) * 0.8
        return depth_map, confidence_map

    def process_keyframe(
        self,
        ref_idx: int,
        keyframes: List[Keyframe]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Main pipeline entry point: computes depth and confidence for keyframe at ref_idx.
        """
        neighbor_indices = self.select_neighbor_keyframes(ref_idx, keyframes)
        if not neighbor_indices:
            H, W = keyframes[ref_idx].image.shape[:2]
            return np.zeros((H, W), dtype=np.float32), np.zeros((H, W), dtype=np.float32)

        src_kfs = [keyframes[i] for i in neighbor_indices]
        ref_kf = keyframes[ref_idx]

        if self.method == "plane_sweep":
            depth, conf = self.estimate_depth_plane_sweep(ref_kf, src_kfs)
        elif self.method == "sgbm":
            depth, conf = self.estimate_depth_sgbm(ref_kf, src_kfs[0])
        else: # Hybrid
            depth_ps, conf_ps = self.estimate_depth_plane_sweep(ref_kf, src_kfs)
            depth_sg, conf_sg = self.estimate_depth_sgbm(ref_kf, src_kfs[0])
            depth = np.where(conf_ps >= conf_sg, depth_ps, depth_sg)
            conf = np.maximum(conf_ps, conf_sg)

        # Attach to keyframe
        ref_kf.depth_map = depth
        ref_kf.confidence_map = conf
        return depth, conf
