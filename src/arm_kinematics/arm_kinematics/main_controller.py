import numpy as np
from arm_kinematics.workspace_memory import load_db, add_entry
from arm_kinematics.validator import is_valid_fk, within_joint_limits
from arm_kinematics.ik_solver import solve_ik
from arm_kinematics.fk_solver import forward_kinematics
from scipy.spatial import KDTree

# your functions


# joint limits (example)
JOINT_LIMITS = [
    (-180, 180),
    (-90, 90),
    (-90, 90),
    (-180, 180),
    (-120, 120),
    (-360, 360),
]

db = load_db()

def build_tree(db):
    if len(db) == 0:
        return None
    return KDTree(db[:, 0:3])

tree = build_tree(db)

def process_target(target):
    global db, tree

    # Step 1: quick lookup
    if tree is not None:
        dist, _ = tree.query(target)
        if dist < 0.01:
            print("⚡ Known reachable point")

    # Step 2: solve IK
    angles = solve_ik(target)

    if angles is None:
        print("❌ IK failed")
        return

    # Step 3: joint limit check
    if not within_joint_limits(angles, JOINT_LIMITS):
        print("❌ Joint limit violation")
        return

    # Step 4: FK verification
    if not is_valid_fk(forward_kinematics, angles, target):
        print("❌ FK mismatch")
        return

    # Step 5: store learning
    db = add_entry(db, target, angles)
    tree = build_tree(db)

    print("✅ Learned new reachable point")

    return angles