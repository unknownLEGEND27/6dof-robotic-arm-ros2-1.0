import numpy as np

def is_valid_fk(fk_func, angles, target, tol=1e-3):
    pred = fk_func(angles)
    error = np.linalg.norm(np.array(pred) - np.array(target))
    return error < tol

def within_joint_limits(angles, limits):
    for a, (low, high) in zip(angles, limits):
        if not (low <= a <= high):
            return False
    return True