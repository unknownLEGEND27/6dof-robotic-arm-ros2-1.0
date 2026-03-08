import numpy as np
from .dh_transform import dh_matrix

def forward_kinematics(joint_angles, dh_params):

    T = np.identity(4)

    for i in range(len(joint_angles)):

        theta = joint_angles[i] + dh_params[i][0]
        d = dh_params[i][1]
        a = dh_params[i][2]
        alpha = dh_params[i][3]

        T_i = dh_matrix(theta, d, a, alpha)
        T = np.dot(T, T_i)

    position = T[0:3, 3]

    return T, position