import pytest
import numpy as np
from scipy.spatial.transform import Rotation

from trajectory.camera_pose import (
    pose_to_matrix,
    matrix_to_pose,
    BodyPose,
    CameraExtrinsics,
    transform_body_to_camera_pose,
)


def test_pure_translation_forward():
    """
    Step 2.5 Numerical Test:
    Drone position p_WB = [1.0, 2.0, 3.0]
    Identity rotation R_WB = Eye(3)
    Camera 0.1m forward of body: p_BC = [0.1, 0.0, 0.0], R_BC = Eye(3)
    Expected p_WC = [1.1, 2.0, 3.0]
    """
    p_WB = np.array([1.0, 2.0, 3.0])
    R_WB = np.eye(3)
    p_BC = np.array([0.1, 0.0, 0.0])
    R_BC = np.eye(3)

    body = BodyPose(timestamp=0.0, p_WB=p_WB, R_WB=R_WB)
    extr = CameraExtrinsics(R_BC=R_BC, p_BC=p_BC)

    cam_pose = transform_body_to_camera_pose(body, extr)

    expected_p_WC = np.array([1.1, 2.0, 3.0])
    expected_R_WC = np.eye(3)

    np.testing.assert_allclose(cam_pose.p_WC, expected_p_WC, atol=1e-12)
    np.testing.assert_allclose(cam_pose.R_WC, expected_R_WC, atol=1e-12)


def test_rotated_drone_translation():
    """
    Drone rotated 90 degrees around Z-axis (Yaw = +90 deg):
    R_WB = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    p_WB = [0.0, 0.0, 1.0]
    Camera 0.1m forward in Body frame (+X_B): p_BC = [0.1, 0.0, 0.0]
    Since drone faces +Y_W, camera should be at [0.0, 0.1, 1.0] in World frame!
    """
    R_WB = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    p_WB = np.array([0.0, 0.0, 1.0])
    p_BC = np.array([0.1, 0.0, 0.0])
    R_BC = np.eye(3)

    body = BodyPose(timestamp=1.0, p_WB=p_WB, R_WB=R_WB)
    extr = CameraExtrinsics(R_BC=R_BC, p_BC=p_BC)

    cam_pose = transform_body_to_camera_pose(body, extr)

    expected_p_WC = np.array([0.0, 0.1, 1.0])
    np.testing.assert_allclose(cam_pose.p_WC, expected_p_WC, atol=1e-12)
    np.testing.assert_allclose(cam_pose.R_WC, R_WB, atol=1e-12)


def test_matrix_multiplication_equivalence():
    """
    Verifies that 4x4 matrix multiplication:
    T_WC = T_WB @ T_BC
    is strictly identical to:
    R_WC = R_WB @ R_BC
    p_WC = p_WB + R_WB @ p_BC
    """
    np.random.seed(42)
    for _ in range(20):
        # Random body pose
        p_WB = np.random.uniform(-10.0, 10.0, size=3)
        R_WB = Rotation.from_rotvec(np.random.normal(0, 1, 3)).as_matrix()

        # Random camera extrinsics
        p_BC = np.random.uniform(-0.5, 0.5, size=3)
        R_BC = Rotation.from_rotvec(np.random.normal(0, 1, 3)).as_matrix()

        body = BodyPose(timestamp=10.0, p_WB=p_WB, R_WB=R_WB)
        extr = CameraExtrinsics(R_BC=R_BC, p_BC=p_BC)

        cam_pose = transform_body_to_camera_pose(body, extr)

        # 4x4 Matrix multiplication
        T_WB = body.to_matrix()
        T_BC = extr.to_matrix()
        T_WC_matrix = T_WB @ T_BC

        R_WC_mat, p_WC_mat = matrix_to_pose(T_WC_matrix)

        np.testing.assert_allclose(cam_pose.R_WC, R_WC_mat, atol=1e-12)
        np.testing.assert_allclose(cam_pose.p_WC, p_WC_mat, atol=1e-12)
        np.testing.assert_allclose(cam_pose.to_matrix(), T_WC_matrix, atol=1e-12)


def test_inverse_transformation():
    """
    Verifies that T_WC.inverse() @ T_WC = Eye(4)
    """
    p_WB = np.array([2.5, -1.2, 3.4])
    R_WB = Rotation.from_euler('xyz', [15, -30, 45], degrees=True).as_matrix()
    p_BC = np.array([0.15, -0.05, 0.02])
    R_BC = Rotation.from_euler('xyz', [0, 90, -90], degrees=True).as_matrix()

    body = BodyPose(timestamp=0.5, p_WB=p_WB, R_WB=R_WB)
    extr = CameraExtrinsics(R_BC=R_BC, p_BC=p_BC)
    cam_pose = transform_body_to_camera_pose(body, extr)

    T_WC = cam_pose.to_matrix()
    T_CW = np.linalg.inv(T_WC)

    identity_check = T_CW @ T_WC
    np.testing.assert_allclose(identity_check, np.eye(4), atol=1e-12)
