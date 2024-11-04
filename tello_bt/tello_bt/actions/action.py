import py_trees
from plugin_services.srv import PluginServer
from rclpy.node import Node
from rclpy.client import Client
from rclpy.logging import rclpy

STATUS_MAP = ["FAILURE", "RUNNING", "SUCCESS"]


class Action(py_trees.behaviour.Behaviour):
    """Default py_trees behaviour with ros2 Node

    Use this in case you need to work with anything ros2 related
    """

    def __init__(self, name, bt_node: Node):
        super().__init__(name)
        self.node = bt_node
