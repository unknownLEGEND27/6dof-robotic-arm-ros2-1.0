import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
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

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.dh_params = [
        [0,0.160,0.150,1.57],
        [1.57,0.0,0.350,0],
        [0,0.0,-0.045,1.57],
        [0,0.361,0,-1.57],
        [0,0.0,0,1.57],
        [0,0.104,0.0,0]
        ]

        self.q_current = [0,0,0,0,0,0]


    def target_callback(self,msg):

        target = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z
        ])

        q_solution = inverse_kinematics(
            target,
            self.q_current,
            self.dh_params
        )

        self.q_current = q_solution

        joint_msg = JointState()

        joint_msg.header.stamp = self.get_clock().now().to_msg()

        joint_msg.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6"
        ]

        joint_msg.position = q_solution.tolist()

        joint_msg.velocity = [0.0]*6
        joint_msg.effort = [0.0]*6

        self.joint_pub.publish(joint_msg)

        self.get_logger().info(f"IK Solution: {q_solution}")


def main(args=None):

    rclpy.init(args=args)

    node = IKNode()

    rclpy.spin(node)

    rclpy.shutdown()