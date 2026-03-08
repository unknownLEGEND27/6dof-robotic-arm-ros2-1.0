import numpy as np
from .fk_solver import forward_kinematics


def numerical_jacobian(q, dh_params, delta=1e-6):

    n = len(q)
    J = np.zeros((3, n))

    T, pos = forward_kinematics(q, dh_params)

    for i in range(n):

        dq = q.copy()
        dq[i] += delta

        _, pos_d = forward_kinematics(dq, dh_params)

        J[:, i] = (pos_d - pos) / delta

    return J


def inverse_kinematics(target_pos, q_init, dh_params, max_iter=1000, alpha=0.3):

    q = np.array(q_init)

    for _ in range(max_iter):

        T, pos = forward_kinematics(q, dh_params)

        error = target_pos - pos

        if np.linalg.norm(error) < 1e-3:
            break

        J = numerical_jacobian(q, dh_params)

        dq = alpha * np.linalg.pinv(J).dot(error)

        q = q + dq

    return q