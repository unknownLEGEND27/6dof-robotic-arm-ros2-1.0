import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np

from .fk_solver import forward_kinematics

class FKNode(Node):

    def __init__(self):
        super().__init__('fk_node')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)

        self.pose_pub = self.create_publisher(Pose, '/ee_pose', 10)

        self.dh_params = [
            [0, 160, 150, 1.57],
            [1.57, 0.0, 350, 0],
            [0, 0.0, -45, 1.57],
            [0, 361, 0, -1.57],
            [0, 0.0, 0, 1.57],
            [0, 104, 0.0, 0]
        ]

    def joint_callback(self, msg):

        joints = list(msg.position)

        T, pos = forward_kinematics(joints, self.dh_params)

        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]

        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    rclpy.shutdown()