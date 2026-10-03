"""
Trajectory and Camera Pose Management Module for Phase 2.
"""

from .camera_pose import (
    BodyPose,
    CameraExtrinsics,
    CameraPose,
    CameraTrajectory,
    transform_body_to_camera_pose,
    matrix_to_pose,
    pose_to_matrix,
)
from .trajectory_exporter import TrajectoryExporter
from .trajectory_metrics import (
    compute_camera_trajectory_metrics,
    plot_camera_trajectory_comparison,
)

__all__ = [
    "BodyPose",
    "CameraExtrinsics",
    "CameraPose",
    "CameraTrajectory",
    "transform_body_to_camera_pose",
    "matrix_to_pose",
    "pose_to_matrix",
    "TrajectoryExporter",
    "compute_camera_trajectory_metrics",
    "plot_camera_trajectory_comparison",
]
