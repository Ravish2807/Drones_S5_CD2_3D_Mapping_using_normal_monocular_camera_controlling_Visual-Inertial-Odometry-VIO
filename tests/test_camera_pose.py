import pytest
import numpy as np
from scipy.spatial.transform import Rotation

from core.state import VIOState
from core.math_utils import rot_to_quat
from trajectory.camera_pose import (
    BodyPose,
    CameraExtrinsics,
    CameraPose,
    CameraTrajectory,
    transform_body_to_camera_pose,
)


def test_body_pose_from_vio_state():
    """Verifies BodyPose is correctly created from VIOState."""
    pos = np.array([1.0, 2.0, 3.0])
    vel = np.array([0.1, -0.2, 0.5])
    R = Rotation.from_euler('xyz', [10, 20, 30], degrees=True).as_matrix()
    q = rot_to_quat(R)
    ba = np.array([0.01, -0.01, 0.02])
    bg = np.array([0.001, -0.002, 0.001])
    cov_diag = np.ones(15) * 1e-4

    state = VIOState(
        timestamp=1.25,
        position=pos,
        velocity=vel,
        orientation_R=R,
        orientation_q=q,
        bias_accel=ba,
        bias_gyro=bg,
        covariance_diag=cov_diag
    )

    body_pose = BodyPose.from_vio_state(state)
    assert body_pose.timestamp == 1.25
    np.testing.assert_allclose(body_pose.p_WB, pos)
    np.testing.assert_allclose(body_pose.R_WB, R)
    np.testing.assert_allclose(body_pose.v_WB, vel)
    np.testing.assert_allclose(body_pose.q_wxyz, q)


def test_quaternion_conventions():
    """Verifies that q_xyzw and q_wxyz return expected formats."""
    R = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    cam = CameraPose(timestamp=0.0, p_WC=np.zeros(3), R_WC=R)

    q_xyzw = cam.q_xyzw
    q_wxyz = cam.q_wxyz

    # Check scalar-last vs scalar-first relationship
    assert len(q_xyzw) == 4
    assert len(q_wxyz) == 4
    assert np.isclose(q_wxyz[0], q_xyzw[3]) # w
    assert np.isclose(q_wxyz[1], q_xyzw[0]) # x
    assert np.isclose(q_wxyz[2], q_xyzw[1]) # y
    assert np.isclose(q_wxyz[3], q_xyzw[2]) # z


def test_camera_trajectory_interpolation():
    """Tests SLERP and linear interpolation along camera trajectory."""
    p0 = np.array([0.0, 0.0, 0.0])
    p1 = np.array([2.0, 0.0, 0.0])
    R0 = np.eye(3)
    R1 = Rotation.from_euler('z', 90, degrees=True).as_matrix()

    c0 = CameraPose(timestamp=0.0, p_WC=p0, R_WC=R0)
    c1 = CameraPose(timestamp=1.0, p_WC=p1, R_WC=R1)

    traj = CameraTrajectory([c0, c1])
    assert len(traj) == 2

    # Midpoint at t=0.5
    mid = traj.interpolate_pose(0.5)
    assert mid is not None
    assert mid.timestamp == 0.5
    np.testing.assert_allclose(mid.p_WC, np.array([1.0, 0.0, 0.0]), atol=1e-12)

    # Rotation midpoint should be 45 degrees yaw
    expected_R_mid = Rotation.from_euler('z', 45, degrees=True).as_matrix()
    np.testing.assert_allclose(mid.R_WC, expected_R_mid, atol=1e-12)


def test_timestamp_alignment_validation():
    """Tests Step 2.9 timestamp alignment checking."""
    cam_poses = [
        CameraPose(timestamp=0.05 * i, p_WC=np.zeros(3), R_WC=np.eye(3))
        for i in range(10)
    ]
    traj = CameraTrajectory(cam_poses)

    # Identical image timestamps
    img_timestamps = [0.05 * i for i in range(10)]
    report = traj.validate_timestamp_alignment(img_timestamps, max_allowed_dt=1e-5)

    assert report["total_frames"] == 10
    assert report["matched_frames"] == 10
    assert report["is_strictly_synchronized"] is True
    assert report["max_delta_t_sec"] < 1e-12


def test_trajectory_exporter_csv_and_tum(tmp_path):
    """Tests exporting and re-loading camera trajectories in CSV and TUM formats."""
    from trajectory.trajectory_exporter import TrajectoryExporter

    poses = [
        CameraPose(
            timestamp=float(i) * 0.1,
            p_WC=np.array([float(i), float(i)*0.5, 1.2]),
            R_WC=Rotation.from_euler('xyz', [i*2, i*3, i*4], degrees=True).as_matrix()
        )
        for i in range(5)
    ]
    traj = CameraTrajectory(poses)

    # 1. Test CSV format
    csv_file = str(tmp_path / "camera_trajectory.csv")
    TrajectoryExporter.export_csv(traj, csv_file)
    loaded_csv_traj = TrajectoryExporter.load_csv(csv_file)

    assert len(loaded_csv_traj) == 5
    np.testing.assert_allclose(loaded_csv_traj.timestamps, traj.timestamps, atol=1e-6)
    np.testing.assert_allclose(loaded_csv_traj.positions, traj.positions, atol=1e-6)

    # 2. Test TUM format
    tum_file = str(tmp_path / "camera_trajectory.tum")
    TrajectoryExporter.export_tum(traj, tum_file)
    loaded_tum_traj = TrajectoryExporter.load_tum(tum_file)

    assert len(loaded_tum_traj) == 5
    np.testing.assert_allclose(loaded_tum_traj.timestamps, traj.timestamps, atol=1e-6)
    np.testing.assert_allclose(loaded_tum_traj.positions, traj.positions, atol=1e-6)

