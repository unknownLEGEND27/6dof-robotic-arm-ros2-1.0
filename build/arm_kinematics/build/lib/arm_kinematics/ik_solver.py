import numpy as np
from .fk_solver import forward_kinematics


# =========================
# Jacobian (6xN)
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

        # Orientation diff (skew approx)
        # Orientation diff (BETTER VERSION)
        dR = R_d @ R.T

        dphi = np.array([
            dR[2,1] - dR[1,2],
            dR[0,2] - dR[2,0],
            dR[1,0] - dR[0,1]
        ]) * 0.5 / delta

        J[:, i] = np.concatenate((dp, dphi))

    return J


# =========================
# IK Solver (Quaternion-based)
# =========================
# def inverse_kinematics(target_pos, target_quat, q_init, dh_params,
#                       max_iter=500, alpha=0.1):

#     q = np.array(q_init)

#     # 🔥 Normalize target quaternion
#     target_quat = target_quat / np.linalg.norm(target_quat)

#     for _ in range(max_iter):

#         T, pos = forward_kinematics(q, dh_params)
#         R = T[:3, :3]

#         # Position error
#         pos_error = target_pos - pos

#         # Orientation error
#         q_current = rot_to_quat(R)
#         q_current = q_current / np.linalg.norm(q_current)

#         # 🔥 Prevent quaternion flipping
#         if np.dot(target_quat, q_current) < 0:
#             target_quat = -target_quat

#         q_error = quat_multiply(target_quat, quat_conjugate(q_current))

#         ori_error = q_error[:3]   # vector part

#         # 🔥 Scale orientation (IMPORTANT)
#         ori_error *= 0.5

#         # Combined error
#         error = np.concatenate((pos_error, ori_error))

#         if np.linalg.norm(error) < 1e-3:
#             break

#         J = numerical_jacobian(q, dh_params)

#         dq = alpha * np.linalg.pinv(J).dot(error)

#         q = q + dq

#     return q

def inverse_kinematics(target_pos, target_quat, q_init, dh_params,
                      max_iter=500, alpha=0.5):

    q = np.array(q_init)

    target_quat = target_quat / np.linalg.norm(target_quat)

    for _ in range(max_iter):

        T, pos = forward_kinematics(q, dh_params)
        R = T[:3, :3]

        # -------------------------
        # Position error
        # -------------------------
        pos_error = target_pos - pos

        # -------------------------
        # Orientation error (FIXED)
        # -------------------------
        q_current = rot_to_quat(R)
        q_current = q_current / np.linalg.norm(q_current)

        # DO NOT modify target_quat
        if np.dot(target_quat, q_current) < 0:
            q_current = -q_current

        q_error = quat_multiply(target_quat, quat_conjugate(q_current))

        # Proper angular error
        ori_error = 2.0 * q_error[:3]

        # -------------------------
        # Combine
        # -------------------------
        error = np.concatenate((pos_error, ori_error))

        if np.linalg.norm(error) < 1e-4:
            break

        # -------------------------
        # Jacobian
        # -------------------------
        J = numerical_jacobian(q, dh_params)

        # -------------------------
        # DAMPED LEAST SQUARES (CRITICAL FIX)
        # -------------------------
        lambda_ = 0.01
        JT = J.T
        J_damped = JT @ np.linalg.inv(J @ JT + lambda_**2 * np.eye(6))

        dq = alpha * (J_damped @ error)

        # -------------------------
        # Update
        # -------------------------
        q = q + dq

        # Normalize angles (prevent explosion)
        q = (q + np.pi) % (2 * np.pi) - np.pi

    return q


# =========================
# Rotation → Quaternion (robust)
# =========================
def rot_to_quat(R):

    trace = np.trace(R)

    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2,1] - R[1,2]) / s
        qy = (R[0,2] - R[2,0]) / s
        qz = (R[1,0] - R[0,1]) / s
    else:
        # fallback cases
        if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            s = np.sqrt(1 + R[0,0] - R[1,1] - R[2,2]) * 2
            qw = (R[2,1] - R[1,2]) / s
            qx = 0.25 * s
            qy = (R[0,1] + R[1,0]) / s
            qz = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s = np.sqrt(1 + R[1,1] - R[0,0] - R[2,2]) * 2
            qw = (R[0,2] - R[2,0]) / s
            qx = (R[0,1] + R[1,0]) / s
            qy = 0.25 * s
            qz = (R[1,2] + R[2,1]) / s
        else:
            s = np.sqrt(1 + R[2,2] - R[0,0] - R[1,1]) * 2
            qw = (R[1,0] - R[0,1]) / s
            qx = (R[0,2] + R[2,0]) / s
            qy = (R[1,2] + R[2,1]) / s
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