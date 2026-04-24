# 6-DOF Robotic Arm — ROS 2 Kinematics Project

**Jenil Patel &nbsp;|&nbsp; Robotics & Automation Engineering**

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?logo=ubuntu)
![Python](https://img.shields.io/badge/Python-3.10-blue?logo=python)
![NumPy](https://img.shields.io/badge/NumPy-scientific-lightgrey)
![SciPy](https://img.shields.io/badge/SciPy-interpolation-lightgrey)

---

## Overview

A fully-featured 6-DOF robotic arm simulation built on **ROS 2 Humble**, featuring:

- **Forward Kinematics (FK)** — DH-parameter chain to compute end-effector pose from joint angles
- **Inverse Kinematics (IK)** — Numerical solver using damped least-squares Jacobian (λ = 0.01)
- **Trajectory Planning** — Smooth LSPB interpolation (0.4 s) for single-goal moves; Cartesian cubic-spline interpolation for multi-waypoint paths
- **Interactive GUI** — Tkinter pose GUI with real-time X/Y/Z sliders
- **RViz Visualization** — Full TF transform tree, live 3D model update

---

## What's New (v1.0 → current)

| Area | Change |
|---|---|
| **Trajectory node** | Added `trajectory_node.py` — sole writer of `/joint_states`; replays IK results as smooth LSPB (0.4 s) interpolations |
| **Topic architecture** | IK node now publishes to `/ik_solution` only; trajectory node owns `/joint_states` — eliminates topic conflict |
| **Multi-waypoint paths** | `/trajectory_goal` topic accepts Cartesian waypoints; cubic spline interpolation via `scipy.interpolate.CubicSpline` |
| **Quaternion handling** | Replaced `tf_transformations` with self-contained `quaternion_matrix()` function — fixes NumPy compatibility issues |
| **GUI corrections** | XY axis swap and Y offset corrected at GUI level (not DH level) |
| **Home pose** | Updated to `x=0.615, y=0.0, z=0.550` |
| **IK stability** | Per-step joint angle clipping to 0.1 rad — prevents divergence near singularities |
| **Dependencies** | `scipy` added (`sudo apt install python3-scipy`) |

---

## System Architecture

```
Target Pose  (/target_pose)
        ↓
   [ IK Node ]  →  /ik_solution
        ↓
[ Trajectory Node ]  →  /joint_states   ← sole writer
        ↓
[ Robot State Publisher ]  →  /tf
        ↓
     [ RViz ]         [ FK Node ]  →  /ee_pose
```

**Node responsibilities:**

| Node | Role |
|---|---|
| `ik_node.py` | Solves IK, publishes to `/ik_solution` |
| `trajectory_node.py` | Subscribes to `/ik_solution`, publishes smooth joint trajectories to `/joint_states` |
| `fk_node.py` | Reads `/joint_states`, publishes end-effector pose to `/ee_pose` |
| `pose_gui_node.py` | Tkinter GUI, publishes target poses to `/target_pose` |
| `robot_state_publisher` | URDF + TF tree from `/joint_states` |

---

## Workspace Structure

```
arm_ws/
  └── src/
       ├── dof6arm/
       │     ├── urdf/              # Robot URDF model
       │     ├── meshes/            # 3D mesh files
       │     ├── rviz/              # RViz config
       │     └── launch/
       ├── arm_kinematics/
       │     ├── dh_transform.py    # DH matrix computation
       │     ├── fk_solver.py       # FK chain T01*T12*...*T56
       │     ├── fk_node.py         # ROS 2 FK node
       │     ├── ik_solver.py       # Damped least-squares IK
       │     ├── ik_node.py         # ROS 2 IK node → /ik_solution
       │     ├── trajectory_node.py # Trajectory planner → /joint_states
       │     └── pose_gui_node.py   # Tkinter GUI
       └── arm_bringup/
             └── launch/
                   └── system.launch.py
```

---

## Requirements & Installation

**OS:** Ubuntu 22.04 &nbsp;|&nbsp; **ROS 2:** Humble

### ROS 2 Dependencies

```bash
sudo apt install ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-rviz2
```

### Python Dependencies

```bash
sudo apt install python3-scipy
```

### Build

```bash
cd ~/arm_ws
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Clean rebuild** (recommended after major changes):

```bash
rm -rf ~/arm_ws/build ~/arm_ws/install
cd ~/arm_ws && colcon build
source /opt/ros/humble/setup.bash && source install/setup.bash
```

---

## Running the System

### Option A — Single Launch (Recommended)

```bash
ros2 launch arm_bringup system.launch.py
```

Starts all nodes: robot state publisher, FK node, IK node, trajectory node, RViz, and the pose GUI.

### Option B — Manual (Multiple Terminals)

**Terminal 1 — Robot Model + RViz**
```bash
source /opt/ros/humble/setup.bash && source ~/arm_ws/install/setup.bash
ros2 launch dof6arm view.launch1.py
```

**Terminal 2 — FK Node**
```bash
ros2 run arm_kinematics fk_node
```

**Terminal 3 — IK Node**
```bash
ros2 run arm_kinematics ik_node
```

**Terminal 4 — Trajectory Node**
```bash
ros2 run arm_kinematics trajectory_node
```

**Terminal 5 — Pose GUI**
```bash
ros2 run arm_kinematics pose_gui_node
```

**CLI — Send a target pose directly**
```bash
ros2 topic pub -r 5 /target_pose geometry_msgs/Pose \
  "{position: {x: 0.3, y: 0.1, z: 0.4}, \
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

> **Note:** Always include the orientation field. An all-zero quaternion will cause IK to fail. Use `w: 1.0` for no rotation.

---

## ROS 2 Topics

| Topic | Type | Publisher | Description |
|---|---|---|---|
| `/target_pose` | `geometry_msgs/Pose` | GUI / CLI | Desired end-effector position + orientation |
| `/ik_solution` | `sensor_msgs/JointState` | `ik_node` | Raw IK result (joint angles) |
| `/joint_states` | `sensor_msgs/JointState` | `trajectory_node` | Smoothed joint angles for robot state publisher |
| `/ee_pose` | `geometry_msgs/Pose` | `fk_node` | FK-computed end-effector pose |
| `/trajectory_goal` | `geometry_msgs/PoseArray` | CLI / external | Multi-waypoint Cartesian path |
| `/tf` | `tf2_msgs/TFMessage` | `robot_state_publisher` | Robot transform tree |

---

## Kinematics Details

### Forward Kinematics

```
T_06 = T_01 × T_12 × T_23 × T_34 × T_45 × T_56
```

Each joint transform uses the standard DH matrix with parameters `(theta, d, a, alpha)`. The result is a 4×4 homogeneous transform of the end-effector relative to the base frame.

### Inverse Kinematics

Numerical IK using damped least-squares Jacobian:

- Numerical Jacobian with finite differences (δ = 1e-6)
- 6-element error vector: [position error (3), rotation error (3)]
- Update rule: `dq = (JᵀJ + λI)⁻¹ Jᵀ e` &nbsp;&nbsp;(λ = 0.01)
- Per-step clipping to **0.1 rad** — prevents joint jumps near singularities
- Convergence: `||error|| < 1e-4`, max 200 iterations

### Trajectory Planning

**Single goal** (`/ik_solution` → `/joint_states`):
LSPB (Linear Segment with Parabolic Blends) interpolation over 0.4 seconds.

**Multi-waypoint** (`/trajectory_goal`):
Cartesian cubic spline interpolation (`scipy.interpolate.CubicSpline`) across waypoints, with LSPB joint-space fallback.

### DH Parameters

| Joint | θ offset (rad) | d (m) | a (m) | α (rad) |
|---|---|---|---|---|
| 1 | 0 | 0.160 | 0.150 | 1.57 |
| 2 | 1.57 | 0.000 | 0.350 | 0 |
| 3 | 0 | 0.000 | −0.045 | 1.57 |
| 4 | 0 | 0.361 | 0 | −1.57 |
| 5 | 0 | 0.000 | 0 | 1.57 |
| 6 | 0 | 0.104 | 0 | 0 |

### Joint Limits (enforced after every IK iteration)

| Joint | Limit |
|---|---|
| Joint 1 | ±150° |
| Joints 2, 3, 5 | ±80° |
| Joints 4, 6 | ±180° |

---

## Debugging

| Command | Purpose |
|---|---|
| `ros2 node list` | List all active nodes |
| `ros2 topic list` | List all active topics |
| `ros2 topic echo /ik_solution` | Monitor raw IK output |
| `ros2 topic echo /joint_states` | Monitor smoothed joint angles |
| `ros2 topic echo /ee_pose` | Monitor end-effector pose |
| `ros2 run tf2_tools view_frames` | Generate TF tree diagram (PDF) |
| `ros2 run tf2_ros tf2_echo base_link tool0` | Inspect specific TF transforms |

---

## Known Constraints & Design Decisions

- **DH parameters are locked** — changes destabilize IK; all positional corrections are applied at the GUI or node level
- **`trajectory_node` is the sole writer of `/joint_states`** — do not publish to this topic from any other node
- **colcon caches aggressively** — run a clean rebuild (`rm -rf build install`) when changes aren't being picked up
- **Orientation convention** — ZYX intrinsic (ROS standard RPY); quaternions used via `geometry_msgs/Quaternion` to avoid axis-order ambiguity

---

## Future Improvements

- Analytical IK solver for faster, closed-form solutions
- MoveIt2 integration for motion planning and collision avoidance
- Gazebo simulation with physics and sensor feedback
- Web-based robot control interface
- Obstacle avoidance using occupancy maps
- Gripper control and end-effector state management

---

## Author

**Jenil Patel**  
Robotics & Automation Engineering
