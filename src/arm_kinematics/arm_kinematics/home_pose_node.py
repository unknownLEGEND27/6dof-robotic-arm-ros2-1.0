import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class HomePosePublisher(Node):

    HOME_POSE = {
        'x': 0.615, 'y': 0.0, 'z': 0.550,
        'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0,
    }

    def __init__(self):
        super().__init__('home_pose_publisher')

        self.publisher = self.create_publisher(Pose, '/target_pose', 10)

        # Non-blocking 3 s delay — lets IK node finish starting up first.
        self.timer = self.create_timer(0.5, self._publish_once)
        self._published = False

    def _publish_once(self):
        if self._published:
            return
        self._published = True

        msg = Pose()
        msg.position.x = self.HOME_POSE['x']
        msg.position.y = self.HOME_POSE['y']
        msg.position.z = self.HOME_POSE['z']
        msg.orientation.x = self.HOME_POSE['qx']
        msg.orientation.y = self.HOME_POSE['qy']
        msg.orientation.z = self.HOME_POSE['qz']
        msg.orientation.w = self.HOME_POSE['qw']

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Home pose published: pos=({msg.position.x}, "
            f"{msg.position.y}, {msg.position.z})"
        )
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = HomePosePublisher()
    try:
        rclpy.spin(node)        # keeps the publisher alive after publish
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()