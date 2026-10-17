#!/usr/bin/env python3
"""
3D Visualization Utilities for VIO-Guided Reconstruction.
Provides 3D camera frustums, object orientation coordinate frames,
bounding boxes, and multi-view trajectory renderers for Open3D and WebGL.
"""

import numpy as np
import open3d as o3d
from typing import List, Optional, Tuple
from .keyframe_bundle import Keyframe, CameraIntrinsics


def create_camera_frustum(
    T_WC: np.ndarray,
    fx: float = 400.0,
    fy: float = 400.0,
    cx: float = 320.0,
    cy: float = 240.0,
    width: int = 640,
    height: int = 480,
    scale: float = 0.25,
    color_rgb: Optional[List[float]] = None
) -> o3d.geometry.LineSet:
    """
    Constructs an Open3D LineSet representing a 3D pinhole camera wireframe frustum.
    Shows the camera apex, 4 corner rays, image sensor rectangle, and optical axis.
    """
    color = color_rgb or [1.0, 0.75, 0.0] # Amber/Gold default

    # 1. 4 corners in Camera coordinate system at depth = scale
    s = float(scale)
    x0 = -cx * s / fx
    x1 = (width - cx) * s / fx
    y0 = -cy * s / fy
    y1 = (height - cy) * s / fy

    # Points in Camera Frame: 0: Apex, 1: Top-Left, 2: Top-Right, 3: Bottom-Right, 4: Bottom-Left
    pts_C = np.array([
        [0.0, 0.0, 0.0], # Apex (Optical Center)
        [x0, y0, s],     # Top-Left
        [x1, y0, s],     # Top-Right
        [x1, y1, s],     # Bottom-Right
        [x0, y1, s],     # Bottom-Left
        [0.0, y0 * 1.3, s] # Top orientation marker tip
    ], dtype=np.float64)

    # 2. Transform to World Frame: P_W = R_WC * P_C + p_WC
    R_WC = T_WC[0:3, 0:3]
    p_WC = T_WC[0:3, 3]
    pts_W = (R_WC @ pts_C.T).T + p_WC

    # 3. Lines connecting frustum:
    # 4 apex rays + 4 rectangle edges + 2 top orientation marker edges
    lines = [
        [0, 1], [0, 2], [0, 3], [0, 4], # Apex to corners
        [1, 2], [2, 3], [3, 4], [4, 1], # Image rectangle
        [1, 5], [5, 2]                  # Top direction triangle
    ]
    colors = [color for _ in range(len(lines))]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(pts_W)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def create_frustums_from_keyframes(
    keyframes: List[Keyframe],
    stride: int = 1,
    scale: float = 0.20,
    color_rgb: Optional[List[float]] = None
) -> List[o3d.geometry.LineSet]:
    """Generates wireframe frustums for a sequence of keyframes."""
    frustums = []
    for idx in range(0, len(keyframes), stride):
        kf = keyframes[idx]
        intr = kf.intrinsics
        f = create_camera_frustum(
            T_WC=kf.T_WC,
            fx=intr.fx,
            fy=intr.fy,
            cx=intr.cx,
            cy=intr.cy,
            width=intr.width,
            height=intr.height,
            scale=scale,
            color_rgb=color_rgb
        )
        frustums.append(f)
    return frustums


def create_object_coordinate_frame(
    centroid: np.ndarray,
    size: float = 0.40,
    bounding_box_extent: Optional[np.ndarray] = None
) -> List[o3d.geometry.Geometry]:
    """
    Creates an explicit 3D coordinate frame (Red=X, Green=Y, Blue=Z)
    and optional bounding box wireframe at the object's estimated position.
    """
    geometries = []
    # 1. 3D Coordinate Axis at Object Centroid
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=centroid)
    geometries.append(axis)

    # 2. Bounding Box wireframe
    if bounding_box_extent is not None:
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=centroid - bounding_box_extent * 0.5,
            max_bound=centroid + bounding_box_extent * 0.5
        )
        bbox.color = [0.9, 0.2, 0.8] # Magenta
        geometries.append(bbox)

    return geometries
