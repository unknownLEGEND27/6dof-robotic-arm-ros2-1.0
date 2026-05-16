# 6DOF Robotic Arm — ROS 2 Simulation & Control

**Jenil Patel** | Robotics & Automation Engineering

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo Ignition](https://img.shields.io/badge/Gazebo-Ignition%20Fortress-FF6F00?logo=gazebo&logoColor=white)](https://gazebosim.org/docs/fortress)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](https://github.com/unknownLEGEND27/6dof-robotic-arm-ros2-1.0/pulls)

> An end-to-end simulation, kinematics, and browser-based teleoperation stack for a 6-degree-of-freedom robotic arm — built on ROS 2 Humble, Gazebo Ignition Fortress, and a Synthwave-themed WebGL control panel.

---

## Overview

A fully-featured 6-DOF robotic arm system covering the complete kinematic pipeline from interactive web GUI to Gazebo Ignition physics simulation. The arm CAD is authored in SolidWorks and exported as URDF/xacro with STL meshes, simulated under Gazebo Ignition Fortress, and driven through the `gz_ros2_control` hardware interface.

- **Forward Kinematics (FK)** — DH-parameter chain; end-effector pose from joint angles
- **Inverse Kinematics (IK)** — damped least-squares Jacobian (λ = 0.01), 200-iteration convergence
- **Hardware interface** — `IgnitionSystem` + `joint_trajectory_controller` + `joint_state_broadcaster`
- **Web control** — pure HTML/JS UI bridged over WebSocket via `rosbridge_server`
- **3D visualization** — live Three.js scene in the browser, with ghost-arm target preview and workspace sphere

---

## What's Changed Since the Earlier Build

| Area              | Change                                                                                         |
| ----------------- | ---------------------------------------------------------------------------------------------- |
| Simulator         | Migrated from RViz-only visualization to **full Gazebo Ignition Fortress** physics simulation  |
| Controller stack  | Added **`gz_ros2_control`** with `joint_trajectory_controller` and `joint_state_broadcaster`    |
| User interface    | Replaced Tkinter GUI with a **standalone web UI** (HTML/CSS/JS + Three.js, no build step)      |
| Bridge            | Added **`rosbridge_server`** WebSocket bridge so the browser talks directly to ROS 2 topics    |
| Trajectory        | Trajectory smoothing now handled by `joint_trajectory_controller`; IK publishes joint targets   |
| Visualization     | Live arm rendering moved into the browser using Three.js (orbit/pan/zoom, ghost target arm)     |
| Programming model | Added in-browser **trajectory builder** and **motion recorder** with save/load JSON            |
| Joint limits      | Updated to ±180° on joints 1, 3, 4, 6 and ±90° on joints 2, 5                                  |

---

## Features

- **Full physics simulation** under Gazebo Ignition Fortress with the `IgnitionSystem` hardware interface
- **Custom kinematics layer** — DH-parameter forward kinematics + damped least-squares numerical IK
- **Cartesian pose control** via `/target_pose` — UI publishes, IK node resolves, controller executes
- **Browser-based control UI** — pure HTML/CSS/JS, no build step, opens directly from disk
- **Three.js 3D visualization** with live arm state, translucent ghost-arm target preview, workspace sphere, orbit camera
- **Trajectory builder** — author Cartesian waypoints in the browser, loop with configurable dwell
- **Motion recorder** — capture poses in real time, replay at variable speed, save and load as JSON
- **Six circular SVG joint gauges** with gradient arcs and color-coded warning/danger zones
- **Preset poses** — Home, Zero, plus four user save slots persisted in the browser
- **Keyboard control** — arrow keys + W/S for jogging, `H` for home, `Enter` to send, `Esc` for e-stop
- **Live telemetry** — frequency, round-trip latency, target-to-current distance shown in the header
- **Synthwave Pro theme** — aurora gradient background, glassmorphism panels, neon accents

---

## System Architecture

### Data Flow

```text
            Web UI (interface.html)
                    │
        WebSocket  │  rosbridge_server (ws://localhost:9090)
                    ▼
              /target_pose  (geometry_msgs/Pose)
                    │
                    ▼
              [ ik_node ]  ──►  /joint_states  (sensor_msgs/JointState)
                                      │
                                      ▼
                    [ joint_trajectory_controller ]
                                      │
                                      ▼
                    Gazebo Ignition Fortress (IgnitionSystem)
                                      │
                                      ▼
                              [ fk_node ]  ──►  /ee_pose  (geometry_msgs/Pose)
                                      │
                                      ▼
                              [ /tf ]  ──►  Three.js live render in browser
```

### Node Responsibilities

| Node                          | Role                                                                                     |
| ----------------------------- | ---------------------------------------------------------------------------------------- |
| `ik_node`                     | Solves IK from `/target_pose`, publishes joint angles to `/joint_states`                  |
| `fk_node`                     | Reads `/joint_states`, computes end-effector pose, publishes to `/ee_pose`                |
| `home_pose_node`              | Drives the arm to a known home configuration on startup                                   |
| `joint_trajectory_controller` | Executes joint targets through the Gazebo Ignition `IgnitionSystem` interface             |
| `joint_state_broadcaster`     | Reads simulator state and broadcasts to `/joint_states` consumers                         |
| `robot_state_publisher`       | URDF + TF transform tree from `/joint_states`                                             |
| `rosbridge_server`            | WebSocket bridge between ROS 2 topics and the browser UI                                  |

---

## Repository Structure

```text
6dof-robotic-arm-ros2-1.0/
├── src/
│   ├── dof6arm/                       # Robot description & simulation package
│   │   ├── urdf/                      # xacro / URDF (SolidWorks export)
│   │   ├── meshes/                    # STL visual & collision meshes
│   │   ├── launch/
│   │   │   └── ignition.launch.py     # Top-level launch (Gazebo + controllers + nodes + UI)
│   │   ├── config/
│   │   │   └── controllers.yaml       # joint_trajectory_controller config
│   │   ├── rviz/
│   │   │   └── arm.rviz               # RViz2 visualization layout
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   └── arm_kinematics/                # Kinematics nodes (Python / rclpy)
│       ├── arm_kinematics/
│       │   ├── dh_transform.py        # DH matrix computation
│       │   ├── fk_solver.py           # FK chain T01·T12·…·T56
│       │   ├── fk_node.py             # ROS 2 FK node → /ee_pose
│       │   ├── ik_solver.py           # Damped least-squares IK
│       │   ├── ik_node.py             # ROS 2 IK node → /joint_states
│       │   └── home_pose_node.py      # Drives arm to home on startup
│       ├── setup.py
│       └── package.xml
│
├── interface.html                     # Standalone web control UI (open in browser)
├── LICENSE
└── README.md
```

---

## Requirements

### System

- **Ubuntu 22.04 LTS**
- **ROS 2 Humble Hawksbill** — [installation guide](https://docs.ros.org/en/humble/Installation.html)
- **Gazebo Ignition Fortress** — [installation guide](https://gazebosim.org/docs/fortress/install)

### ROS 2 Packages

```bash
sudo apt install \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-rosbridge-server \
  ros-humble-rosbridge-suite
```

### Python

- Python 3.10+
- `rclpy` (provided by ROS 2 Humble)
- `numpy`

```bash
pip install numpy
```

### Browser

- Any modern Chromium-based or Firefox browser with WebGL 2 support
- Three.js is loaded from CDN — no local install required

---

## Installation & Build

Clone into a ROS 2 workspace:

```bash
mkdir -p ~/arm_ws/src
cd ~/arm_ws/src
git clone https://github.com/unknownLEGEND27/6dof-robotic-arm-ros2-1.0.git
```

Resolve dependencies and build with `colcon`:

```bash
cd ~/arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Source the overlay (do this in every new terminal):

```bash
source /opt/ros/humble/setup.bash
source ~/arm_ws/install/setup.bash
```

> 💡 **Tip:** Add the two `source` lines to your `~/.bashrc` to skip this step in future sessions.
>
> ⚠️ **Clean rebuild** recommended after major URDF or controller changes:
> ```bash
> rm -rf ~/arm_ws/build ~/arm_ws/install ~/arm_ws/log
> colcon build --symlink-install
> ```

---

## Launch

A single launch file brings up the full stack — Gazebo Ignition, robot spawn, controllers, kinematics nodes, rosbridge, and the browser UI:

```bash
ros2 launch dof6arm ignition.launch.py
```

This sequence:

1. Starts Gazebo Ignition Fortress with an empty world
2. Spawns the 6-DOF arm from URDF/xacro
3. Loads `joint_state_broadcaster` and `joint_trajectory_controller`
4. Starts `ik_node`, `fk_node`, and `home_pose_node`
5. Starts `rosbridge_server` on `ws://localhost:9090`
6. Opens `interface.html` in your default browser

If the UI does not open automatically, open `interface.html` manually:

```bash
xdg-open interface.html
```

---

## Web Interface Guide

The UI is split into three regions plus a header and joint-gauge footer.

### Header

- **FREQ** — current update frequency on `/joint_states`
- **LATENCY** — round-trip time between target publish and joint feedback
- **DIST** — Euclidean distance from current EE pose to target EE pose
- **ROS connection indicator** — green when the rosbridge WebSocket is live

### Left Panel — Pose Input

- **X / Y / Z** position sliders with paired numeric inputs (metres)
- **Roll / Pitch / Yaw** orientation sliders with paired numeric inputs (degrees)
- **Motion tab** — speed scaling, auto-send toggle, workspace-sphere visibility, dwell time
- **Status tab** — live end-effector XYZ readout from `/ee_pose`

### Centre — 3D Visualization

- Three.js scene with the live arm and a translucent **ghost arm** showing the target pose
- Translucent **workspace sphere** outlining the reachable volume
- **Orbit camera** — left-drag to rotate, scroll to zoom, right-drag to pan

### Right Panel — Programming

- **Presets** — Home, Zero, plus four user save slots (persisted in `localStorage`)
- **Trajectory builder** — add, delete, reorder, and execute waypoints with optional loop and per-waypoint dwell
- **Motion recorder** — record poses live, play back at variable speed, loop, save and load as JSON
- **Live EE pose** — XYZ in metres, RPY in degrees

### Footer — Joint Gauges

Six circular SVG gauges, one per joint, showing live position against its limit with gradient arcs and color-coded warning/danger zones.

### Keyboard Shortcuts

| Key            | Action                              |
| -------------- | ----------------------------------- |
| `←` / `→`      | Jog Y axis                          |
| `↑` / `↓`      | Jog Z axis                          |
| `W` / `S`      | Jog X axis                          |
| `H`            | Move to home pose                   |
| `Enter`        | Publish current target pose         |
| `Escape`       | Emergency stop (halts trajectory)   |

---

## ROS 2 Topics

| Topic            | Type                       | Publisher                    | Description                                                |
| ---------------- | -------------------------- | ---------------------------- | ---------------------------------------------------------- |
| `/target_pose`   | `geometry_msgs/Pose`       | Web UI (via rosbridge)        | Desired end-effector position + orientation                |
| `/joint_states`  | `sensor_msgs/JointState`   | `ik_node` / `joint_state_broadcaster` | Joint angles consumed by the controller and UI       |
| `/ee_pose`       | `geometry_msgs/Pose`       | `fk_node`                     | FK-computed end-effector pose                              |
| `/tf`            | `tf2_msgs/TFMessage`       | `robot_state_publisher`       | Robot transform tree                                       |

Inspect any topic from the terminal:

```bash
ros2 topic echo /ee_pose
ros2 topic hz   /joint_states
ros2 topic info /target_pose
```

---

## Kinematics Details

### Forward Kinematics

```text
T_06 = T_01 · T_12 · T_23 · T_34 · T_45 · T_56
```

Each joint transform uses the standard DH matrix with parameters `(θ, d, a, α)`. The result is a 4×4 homogeneous transform of the end-effector relative to the base frame.

### Inverse Kinematics — Damped Least Squares

- Numerical Jacobian via finite differences (`δ = 1e-6`)
- 6-element error vector: `[position_error(3), rotation_error(3)]`
- Update rule: `dq = (Jᵀ·J + λ²·I)⁻¹ · Jᵀ · e` with `λ = 0.01`
- Per-step joint-angle clipping to `0.1 rad` — prevents joint jumps near singularities
- Convergence: `||error|| < 1e-4`, max **200** iterations

### Trajectory Smoothing

Joint-space smoothing is performed by `joint_trajectory_controller` using its built-in spline interpolation between the latest IK target and the current state. Velocity and acceleration limits are configured in `config/controllers.yaml`.

---

## Robot Specifications

### DH Parameters

| Joint | θ offset (rad) | d (m) | a (m)   | α (rad) |
| :---: | :------------: | :---: | :-----: | :-----: |
| 1     | 0              | 0.160 |  0.150  |  1.57   |
| 2     | 1.57           | 0.000 |  0.350  |  0      |
| 3     | 0              | 0.000 | −0.045  |  1.57   |
| 4     | 0              | 0.361 |  0      | −1.57   |
| 5     | 0              | 0.000 |  0      |  1.57   |
| 6     | 0              | 0.104 |  0      |  0      |

> ⚠️ **DH parameters are locked** — empirical attempts to "correct" them destabilized the IK solver. Positional corrections are applied at the GUI / node level instead.

### Joint Limits (enforced after every IK iteration)

| Joint      | Range  |
| ---------- | :----: |
| Joint 1    | ±180°  |
| Joint 2    | ±90°   |
| Joint 3    | ±180°  |
| Joint 4    | ±180°  |
| Joint 5    | ±90°   |
| Joint 6    | ±180°  |

---

## Debugging

| Command                                          | Purpose                              |
| ------------------------------------------------ | ------------------------------------ |
| `ros2 node list`                                 | List all active nodes                |
| `ros2 topic list`                                | List all active topics               |
| `ros2 topic echo /joint_states`                  | Monitor joint angles                 |
| `ros2 topic echo /ee_pose`                       | Monitor end-effector pose            |
| `ros2 topic echo /target_pose`                   | Monitor commanded target pose        |
| `ros2 topic hz /joint_states`                    | Measure publish rate                 |
| `ros2 run tf2_tools view_frames`                 | Generate TF tree diagram (PDF)       |
| `ros2 run tf2_ros tf2_echo base_link tool0`      | Inspect specific TF transform        |
| `ros2 control list_controllers`                  | List active controllers              |
| `ros2 control list_hardware_interfaces`          | List hardware interfaces             |

---

## Known Constraints & Design Decisions

- **DH parameters are locked** — an attempted correction destabilized IK entirely; GUI-level workarounds are preferred over editing the DH table.
- **IK stability requires damping** — pure Moore–Penrose pseudoinverse diverged near singularities; damped least squares (`λ = 0.01`) with per-step clipping (`0.1 rad`) was the fix.
- **Orientation convention** — ZYX intrinsic (ROS standard RPY); `geometry_msgs/Quaternion` is used over the wire to avoid axis-order ambiguity.
- **Unit consistency is critical** — a millimetres-vs-metres mismatch between FK and IK was an early root-cause bug. The whole pipeline is metres + radians end-to-end.
- **No collision-aware planning** — the IK solver targets a single Cartesian pose; there is no MoveIt2 integration and no self-collision or environment-collision checking.
- **No real-hardware driver yet** — only the Ignition simulation backend is wired through `gz_ros2_control`. A `ros2_control` hardware interface for a physical arm has not been implemented.
- **rosbridge is unencrypted** — the default `ws://` connection is plaintext. Do not expose port `9090` to untrusted networks without TLS in front.
- **`colcon` caches aggressively** — run a clean rebuild (`rm -rf build install log`) when changes aren't picked up.
- **UI persists state in `localStorage`** — clearing browser storage will erase preset poses and saved recordings.
- **Tested on Ubuntu 22.04 only** — other distributions and ROS 2 releases (Iron, Jazzy) are likely to need minor adjustments to the controller launch.

---

## Future Improvements

- Analytical (closed-form) IK solver as a fast fallback to the numerical one
- MoveIt2 integration for full motion planning and collision avoidance
- `ros2_control` hardware interface for a physical arm
- Obstacle avoidance using occupancy maps
- Gripper / end-effector models and state management
- Multi-arm and multi-world support with proper namespacing
- Unit tests for the FK / IK math

---

## Contributing

Contributions are welcome. To contribute:

```bash
# Fork the repo, then:
git checkout -b feature/your-feature-name
# make changes, commit, push
git push origin feature/your-feature-name
# open a Pull Request
```

Please keep code style consistent with the existing Python modules and document any new ROS topics or parameters in this README.

---

## License

Released under the **MIT License**. See [`LICENSE`](LICENSE) for the full text.

---

<p align="center">
  <strong>Jenil Patel</strong> — Robotics & Automation Engineering<br>
  Built with ROS 2 Humble · Gazebo Ignition Fortress · Three.js<br>
  <a href="https://github.com/unknownLEGEND27/6dof-robotic-arm-ros2-1.0">github.com/unknownLEGEND27/6dof-robotic-arm-ros2-1.0</a>
</p>
