# 6-DOF Robotic Arm – ROS2 Kinematics Project

## Overview

This project implements a **6-DOF robotic arm simulation in ROS2 (Humble)** with:

* URDF robot model
* Forward Kinematics (FK)
* Inverse Kinematics (IK)
* RViz visualization
* Joint state control

The system computes the **end-effector pose from joint angles (FK)** and **joint angles from target position (IK)**.

---

# System Architecture

```
Target Pose
     ↓
Inverse Kinematics Node
     ↓
Joint Angles (/joint_states)
     ↓
Robot State Publisher
     ↓
TF Transform Tree
     ↓
RViz Robot Visualization
     ↓
Forward Kinematics Node
     ↓
End Effector Pose (/ee_pose)
```

---

# Workspace Structure

```
arm_ws
 └── src
      ├── dof6arm
      │     ├── urdf
      │     ├── meshes
      │     ├── rviz
      │     └── launch
      │
      ├── arm_kinematics
      │     ├── fk_node.py
      │     ├── fk_solver.py
      │     ├── dh_transform.py
      │     ├── ik_node.py
      │     └── ik_solver.py
      │
      └── README.md
```

---

# Requirements

* Ubuntu 22.04
* ROS2 Humble
* Python3
* NumPy

Install dependencies:

```
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-rviz2
```

---

# Build the Workspace

```
cd ~/arm_ws
colcon build
source install/setup.bash
```

---

# Running the System

Open **multiple terminals**.

---

## Terminal 1 — Launch Robot Model

```
source /opt/ros/humble/setup.bash
source ~/arm_ws/install/setup.bash

ros2 launch dof6arm view.launch1.py
```

This starts:

* RViz
* robot_state_publisher
* robot visualization

---

## Terminal 2 — Run Forward Kinematics Node

```
source /opt/ros/humble/setup.bash
source ~/arm_ws/install/setup.bash

ros2 run arm_kinematics fk_node
```

This node:

* subscribes to `/joint_states`
* calculates FK
* publishes `/ee_pose`

---

## Terminal 3 — Run Inverse Kinematics Node

```
source /opt/ros/humble/setup.bash
source ~/arm_ws/install/setup.bash

ros2 run arm_kinematics ik_node
```

This node:

* subscribes to `/target_pose`
* computes joint angles
* publishes `/joint_states`

---

## Terminal 4 — Send Target Pose

Example command:

```
ros2 topic pub -r 5 /target_pose geometry_msgs/Pose \
"{position: {x: 0.3, y: 0.1, z: 0.4}}"
```

The robot will move to the desired pose.

---

# Topics Used

| Topic           | Type                   | Description                   |
| --------------- | ---------------------- | ----------------------------- |
| `/joint_states` | sensor_msgs/JointState | Current joint angles          |
| `/target_pose`  | geometry_msgs/Pose     | Desired end-effector position |
| `/ee_pose`      | geometry_msgs/Pose     | FK computed end-effector pose |
| `/tf`           | tf2_msgs/TFMessage     | Robot transform tree          |

---

# Debugging

## List active nodes

```
ros2 node list
```

---

## List topics

```
ros2 topic list
```

---

## Monitor joint states

```
ros2 topic echo /joint_states
```

---

## Check end-effector pose

```
ros2 topic echo /ee_pose
```

---

## Check TF transforms

```
ros2 topic echo /tf
```

---

## Show TF tree

```
ros2 run tf2_tools view_frames
```

---

# Forward Kinematics

Forward kinematics computes the **end-effector pose from joint angles**.

Transformation chain:

```
T06 = T01 * T12 * T23 * T34 * T45 * T56
```

Each transform is generated using **Denavit–Hartenberg parameters**.

---

# Inverse Kinematics

Inverse kinematics computes **joint angles required to reach a target position**.

Current implementation:

* Numerical IK
* Jacobian pseudo-inverse method

Future improvements:

* Analytical IK
* Orientation control
* trajectory planning

---

# Future Improvements

Planned upgrades:

* Analytical IK solver
* MoveIt integration
* Web-based robot control interface
* Gazebo simulation
* trajectory planning
* obstacle avoidance

---

# Author

Jenil Patel
Robotics & Automation Engineering
