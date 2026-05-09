import numpy as np
from .fk_solver import forward_kinematics


# =========================
# Joint limits used by IK (±π, matches the old modulo-wrap range)
# Note: URDF has tighter limits on some joints; the URDF clamps visually
# in RViz. Using ±π here keeps the IK workspace identical to the
# previous (wrap-based) version, so orientation-only changes don't
# drag the EE position.
# =========================
JOINT_LIMITS = np.array([
    [-np.pi, np.pi],   # joint1
    [-np.pi, np.pi],   # joint2
    [-np.pi, np.pi],   # joint3
    [-np.pi, np.pi],   # joint4
    [-np.pi, np.pi],   # joint5
    [-np.pi, np.pi],   # joint6
])


# =========================
# Jacobian (6xN) — numerical, finite differences
# =========================
def numerical_jacobian(q, dh_params, delta=1e-6):

    n = len(q)
    J = np.zeros((6, n))

    T, pos = forward_kinematics(q, dh_params)
    R = T[:3, :3]

    for i in range(n):

        dq = q.copy()
        dq[i] += delta

        T_d, pos_d = forward_kinematics(dq, dh_params)
        R_d = T_d[:3, :3]

        # Position diff
        dp = (pos_d - pos) / delta

        # Orientation diff — small-angle skew of R_d * R^T
        dR = R_d @ R.T
        dphi = np.array([
            dR[2, 1] - dR[1, 2],
            dR[0, 2] - dR[2, 0],
            dR[1, 0] - dR[0, 1]
        ]) * 0.5 / delta

        J[:, i] = np.concatenate((dp, dphi))

    return J


# =========================
# IK Solver (Damped Least Squares, quaternion-based)
# =========================
def inverse_kinematics(target_pos, target_quat, q_init, dh_params,
                       max_iter=500, alpha=0.5):

    q = np.array(q_init, dtype=float)
    target_quat = target_quat / np.linalg.norm(target_quat)

    for _ in range(max_iter):

        T, pos = forward_kinematics(q, dh_params)
        R = T[:3, :3]

        # Position error
        pos_error = target_pos - pos

        # Orientation error
        q_current = rot_to_quat(R)
        q_current = q_current / np.linalg.norm(q_current)

        # Take the shorter rotation (flip current if dot < 0)
        if np.dot(target_quat, q_current) < 0:
            q_current = -q_current

        q_error = quat_multiply(target_quat, quat_conjugate(q_current))

        # Angular error — sign-corrected so we always take shortest path
        ori_error = 2.0 * np.sign(q_error[3]) * q_error[:3]

        # Combined 6D error
        err = np.concatenate((pos_error, ori_error))

        if np.linalg.norm(err) < 1e-4:
            break

        # Jacobian
        J = numerical_jacobian(q, dh_params)

        # Damped Least Squares:  dq = alpha * J^T (J J^T + lambda^2 I)^-1 e
        lambda_ = 0.01
        JT = J.T
        A = J @ JT + (lambda_ ** 2) * np.eye(6)
        dq = alpha * (JT @ np.linalg.solve(A, err))

        # Update
        q = q + dq

        # Keep joints in ±π range (replaces the old modulo wrap)
        q = np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])

    return q


# =========================
# Rotation → Quaternion (robust)
# =========================
def rot_to_quat(R):

    trace = np.trace(R)

    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

    return np.array([qx, qy, qz, qw])


# =========================
# Quaternion helpers
# =========================
def quat_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])  