import numpy as np


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """
    ROS quaternion:
        x, y, z, w

    Returns:
        R in SO(3)
    """

    q = np.array([qx, qy, qz, qw], dtype=float)

    norm = np.linalg.norm(q)

    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError("Invalid quaternion")

    q /= norm

    x, y, z, w = q

    R = np.array([
        [
            1.0 - 2.0 * (y*y + z*z),
            2.0 * (x*y - z*w),
            2.0 * (x*z + y*w)
        ],
        [
            2.0 * (x*y + z*w),
            1.0 - 2.0 * (x*x + z*z),
            2.0 * (y*z - x*w)
        ],
        [
            2.0 * (x*z - y*w),
            2.0 * (y*z + x*w),
            1.0 - 2.0 * (x*x + y*y)
        ]
    ])

    return R


def rotation_matrix_to_quaternion(R):
    """
    Convert SO(3) rotation matrix to ROS quaternion x,y,z,w.
    """

    trace = np.trace(R)

    if trace > 0.0:

        s = 0.5 / np.sqrt(trace + 1.0)

        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s

    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:

        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])

        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s

    elif R[1, 1] > R[2, 2]:

        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])

        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s

    else:

        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])

        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw])

    q /= np.linalg.norm(q)

    return q


def vee(S):
    """
    Inverse of the hat/skew operator.

             [ 0 -z  y ]
    S =       [ z  0 -x ]
             [-y  x  0 ]

    vee(S) = [x,y,z]
    """

    return np.array([
        S[2, 1],
        S[0, 2],
        S[1, 0]
    ])


def hat(v):
    """
    Vector -> skew-symmetric matrix.
    """

    x, y, z = v

    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0]
    ])


def validate_rotation_matrix(R):

    orthogonality_error = np.linalg.norm(
        R.T @ R - np.eye(3)
    )

    determinant_error = abs(np.linalg.det(R) - 1.0)

    return (
        orthogonality_error < 1e-3
        and determinant_error < 1e-3
    )
