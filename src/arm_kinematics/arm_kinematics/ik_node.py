import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

from .ik_solver import inverse_kinematics


class IKNode(Node):

    def __init__(self):
        super().__init__('ik_node')

        self.target_sub = self.create_subscription(
            Pose,
            '/target_pose',
            self.target_callback,
            10
        )

        # Mode switch: in Gazebo we send JointTrajectory to the controller.
        # In RViz-only mode we publish JointState directly to /joint_states.
        self.declare_parameter('use_trajectory_controller', False)
        self.use_trajectory_controller = self.get_parameter(
            'use_trajectory_controller').value

        if self.use_trajectory_controller:
            self.joint_pub = self.create_publisher(
                JointTrajectory,
                '/joint_trajectory_controller/joint_trajectory',
                10
            )
            self.get_logger().info(
                "IK node mode: GAZEBO (publishing JointTrajectory)")
        else:
            self.joint_pub = self.create_publisher(
                JointState,
                '/joint_states',
                10
            )
            self.get_logger().info(
                "IK node mode: RVIZ-ONLY (publishing JointState)")

        # self.target_sub = self.create_subscription(
        #     Pose,
        #     '/target_pose',
        #     self.target_callback,
        #     10
        # )

        # self.joint_pub = self.create_publisher(
        #     JointState,
        #     '/joint_states',
        #     10
        # )

        self.dh_params = [
            [0, 0.160, 0.150, 1.57],
            [1.57, 0.0, 0.350, 0],
            [0, 0.0, -0.045, 1.57],
            [0, 0.361, 0, -1.57],
            [0, 0.0, 0, 1.57],
            [0, 0.104, 0.0, 0]
        ]

        self.q_current = np.zeros(6)

    def target_callback(self, msg):

        # 🔥 Position
        target_pos = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z
        ])

        # 🔥 Quaternion (NO conversion to Euler)
        target_quat = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

        # 🔥 Normalize quaternion (important)
        norm = np.linalg.norm(target_quat)
        if norm == 0:
            self.get_logger().warn("Received zero quaternion, skipping")
            return
        target_quat = target_quat / norm

        # 🔥 Solve IK (new signature)
        q_solution = inverse_kinematics(
            target_pos,
            target_quat,
            self.q_current,
            self.dh_params
        )

        # Smooth update (prevents jumping)
        self.q_current = 0.8 * self.q_current + 0.2 * q_solution
        joint_names = ["joint1", "joint2", "joint3",
                       "joint4", "joint5", "joint6"]

        if self.use_trajectory_controller:
            # Adaptive duration: scale with biggest joint motion.
            # ~1 rad/s → 90° move takes ~1.5 s, small 5° move takes ~0.1 s.
            max_delta = float(np.max(np.abs(q_solution - self.q_current)))
            duration = max(0.1, min(2.0, max_delta / 1.0))   # clamp 0.1..2.0 s

            traj = JointTrajectory()
            traj.joint_names = joint_names

            point = JointTrajectoryPoint()
            point.positions = q_solution.tolist()
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration % 1) * 1e9)
            traj.points = [point]

            self.joint_pub.publish(traj)
            
        else:
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = joint_names
            joint_msg.position = q_solution.tolist()
            joint_msg.velocity = [0.0] * 6
            joint_msg.effort = [0.0] * 6
            self.joint_pub.publish(joint_msg)

        self.get_logger().info(f"IK Solution: {q_solution}")

        # # 🔥 Publish joint states
        # joint_msg = JointState()
        # joint_msg.header.stamp = self.get_clock().now().to_msg()

        # joint_msg.name = [
        #     "joint1",
        #     "joint2",
        #     "joint3",
        #     "joint4",
        #     "joint5",
        #     "joint6"
        # ]

        # joint_msg.position = q_solution.tolist()
        # joint_msg.velocity = [0.0] * 6
        # joint_msg.effort = [0.0] * 6

        # self.joint_pub.publish(joint_msg)

        # self.get_logger().info(f"IK Solution: {q_solution}")


def main(args=None):
    rclpy.init(args=args)

    node = IKNode()
    rclpy.spin(node)

    rclpy.shutdown()