from rclpy.node import Publisher
from tello_bt.nodes import Action
from std_msgs.msg import Empty
import py_trees


class LandAction(Action):
    land_topic: str = "/land"
    land_pub: Publisher

    def setup(self) -> None:  # type: ignore
        self.land_pub = self.node.create_publisher(Empty, self.land_topic, 1)

    def update(self) -> py_trees.common.Status:
        self.land_pub.publish(Empty())
        return py_trees.common.Status.SUCCESS
