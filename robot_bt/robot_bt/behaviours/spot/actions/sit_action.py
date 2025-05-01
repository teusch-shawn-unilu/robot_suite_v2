from rclpy.node import Publisher
from robot_bt.behaviours.shared.actions import Action
from std_msgs.msg import Empty
import py_trees


class SitAction(Action):
    """
    If battery is low, command Spot to sit (instead of Tello’s land).
    """
    sit_topic: str = "/sit"
    sit_pub: Publisher

    def setup(self) -> None:
        self.sit_pub = self.node.create_publisher(Empty, self.sit_topic, 1)

    def update(self) -> py_trees.common.Status:
        self.sit_pub.publish(Empty())
        return py_trees.common.Status.SUCCESS
