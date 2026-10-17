#!/usr/bin/env python3
"""
Keyframe Data Contract & Bundle Storage.
Represents discrete keyframe snapshots containing synchronized images,
calibrated camera intrinsics, VIO camera optical poses (T_WC), and tracking quality.
"""

import os
import cv2
import json
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Tuple


@dataclass
class CameraIntrinsics:
    """Calibrated camera pinhole model intrinsics."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion: Optional[np.ndarray] = None

    @property
    def K(self) -> np.ndarray:
        """3x3 Camera Calibration Matrix."""
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "fx": float(self.fx),
            "fy": float(self.fy),
            "cx": float(self.cx),
            "cy": float(self.cy),
            "width": int(self.width),
            "height": int(self.height),
            "distortion": self.distortion.tolist() if self.distortion is not None else []
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CameraIntrinsics":
        dist = np.array(data["distortion"], dtype=np.float64) if data.get("distortion") else None
        return cls(
            fx=float(data["fx"]),
            fy=float(data["fy"]),
            cx=float(data["cx"]),
            cy=float(data["cy"]),
            width=int(data.get("width", 640)),
            height=int(data.get("height", 480)),
            distortion=dist
        )


@dataclass
class Keyframe:
    """
    Keyframe snapshot for multi-view stereo 3D reconstruction.
    Stores:
      - frame_id: sequential unique identifier
      - timestamp: capture epoch timestamp (seconds)
      - image: uint8 image matrix (HxW or HxWx3)
      - T_WC: 4x4 rigid transformation (World <- Camera)
      - intrinsics: CameraIntrinsics instance
      - tracking_quality: dictionary storing inlier ratio, tracked feature count, etc.
    """
    frame_id: int
    timestamp: float
    image: np.ndarray
    T_WC: np.ndarray
    intrinsics: CameraIntrinsics
    tracking_quality: Dict[str, Any] = field(default_factory=dict)
    feature_ids: Optional[List[int]] = None
    feature_points: Optional[np.ndarray] = None
    depth_map: Optional[np.ndarray] = None
    confidence_map: Optional[np.ndarray] = None

    def __post_init__(self):
        self.T_WC = np.asarray(self.T_WC, dtype=np.float64)
        assert self.T_WC.shape == (4, 4), f"T_WC must be 4x4 matrix, got {self.T_WC.shape}"

    @property
    def R_WC(self) -> np.ndarray:
        """3x3 Rotation matrix (World <- Camera)."""
        return self.T_WC[0:3, 0:3]

    @property
    def p_WC(self) -> np.ndarray:
        """3D Translation vector [tx, ty, tz] in World coordinates (meters)."""
        return self.T_WC[0:3, 3]

    @property
    def optical_center(self) -> np.ndarray:
        return self.p_WC.copy()

    @property
    def viewing_direction(self) -> np.ndarray:
        """Normalized camera optical viewing axis (Z-axis in Camera frame projected to World)."""
        return self.R_WC @ np.array([0.0, 0.0, 1.0])

    def to_summary_dict(self) -> Dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "timestamp": float(self.timestamp),
            "p_WC": self.p_WC.tolist(),
            "viewing_dir": self.viewing_direction.tolist(),
            "tracking_quality": self.tracking_quality
        }


class KeyframeDatabase:
    """
    Manages collection of Keyframes and exports them to structured disk bundles
    compatible with standard MVS, NeRF, and Open3D reconstruction engines.
    """

    def __init__(self):
        self.keyframes: List[Keyframe] = []

    def add(self, kf: Keyframe):
        self.keyframes.append(kf)

    def __len__(self) -> int:
        return len(self.keyframes)

    def __getitem__(self, idx: int) -> Keyframe:
        return self.keyframes[idx]

    def save_to_disk(self, output_dir: str):
        """
        Exports keyframe bundle to disk:
          - output_dir/images/kf_{frame_id:04d}.png
          - output_dir/manifest.json
          - output_dir/transforms.json (NeRF/COLMAP compatible format)
          - output_dir/trajectory.tum
        """
        img_dir = os.path.join(output_dir, "images")
        os.makedirs(img_dir, exist_ok=True)

        manifest = {
            "count": len(self.keyframes),
            "keyframes": []
        }

        tum_lines = []
        transforms_frames = []

        from scipy.spatial.transform import Rotation

        for kf in self.keyframes:
            # 1. Save Image
            img_filename = f"kf_{kf.frame_id:04d}.png"
            img_path = os.path.join(img_dir, img_filename)
            cv2.imwrite(img_path, kf.image)

            # 2. Compute TUM format: timestamp tx ty tz qx qy qz qw
            q = Rotation.from_matrix(kf.R_WC).as_quat() # [qx, qy, qz, qw]
            tum_line = (
                f"{kf.timestamp:.6f} {kf.p_WC[0]:.6f} {kf.p_WC[1]:.6f} {kf.p_WC[2]:.6f} "
                f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"
            )
            tum_lines.append(tum_line)

            # 3. NeRF/COLMAP standard transform frame
            transforms_frames.append({
                "file_path": f"images/{img_filename}",
                "transform_matrix": kf.T_WC.tolist(),
                "timestamp": kf.timestamp
            })

            # 4. Manifest summary
            manifest["keyframes"].append({
                "frame_id": kf.frame_id,
                "timestamp": kf.timestamp,
                "image_file": img_filename,
                "T_WC": kf.T_WC.tolist(),
                "intrinsics": kf.intrinsics.to_dict(),
                "tracking_quality": kf.tracking_quality
            })

        # Save manifest
        with open(os.path.join(output_dir, "manifest.json"), "w") as f:
            json.dump(manifest, f, indent=2)

        # Save NeRF-style transforms.json
        if len(self.keyframes) > 0:
            intr = self.keyframes[0].intrinsics
            transforms_data = {
                "camera_angle_x": float(2.0 * np.arctan(intr.width / (2.0 * intr.fx))),
                "camera_angle_y": float(2.0 * np.arctan(intr.height / (2.0 * intr.fy))),
                "fl_x": intr.fx,
                "fl_y": intr.fy,
                "cx": intr.cx,
                "cy": intr.cy,
                "w": intr.width,
                "h": intr.height,
                "frames": transforms_frames
            }
            with open(os.path.join(output_dir, "transforms.json"), "w") as f:
                json.dump(transforms_data, f, indent=2)

        # Save TUM trajectory
        with open(os.path.join(output_dir, "trajectory.tum"), "w") as f:
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            f.write("\n".join(tum_lines) + "\n")

        print(f"Keyframe bundle ({len(self.keyframes)} keyframes) saved to: {output_dir}")
