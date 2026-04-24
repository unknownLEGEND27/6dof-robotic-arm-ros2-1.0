import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time


class HomePosePublisher(Node):

    def __init__(self):
        super().__init__('home_pose_publisher')

        self.publisher = self.create_publisher(Pose, '/target_pose', 10)

        time.sleep(3)  # wait for system to start

        msg = Pose()
        msg.position.x = 0.3
        msg.position.y = 0.0
        msg.position.z = 0.4

        self.publisher.publish(msg)

        self.get_logger().info("Home pose published")


def main(args=None):
    rclpy.init(args=args)
    node = HomePosePublisher()
    rclpy.spin_once(node)
    rclpy.shutdown()
