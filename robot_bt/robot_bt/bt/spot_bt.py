import py_trees
from rclpy.node import Node
from robot_bt.behaviours.spot.actions import SitAction, SpotGesturesInterpreterAction
from robot_bt.behaviours.spot.conditions import IsRobotConnected
from robot_bt.behaviours.shared.actions import PluginClient
from robot_bt.behaviours.shared.conditions import CanRunPlugin, IsBatteryLow


class SpotBT(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__("SpotBT")
        self.node = node
        self.build_tree()

    def build_tree(self):
        # 1) Connectivity check
        robot_connection = py_trees.composites.Selector(
            "RobotConnection",
            memory=False,
            children=[
                IsRobotConnected("IsRobotConnected"),
            ],
        )

        # 2) Battery check → sit if low
        battery_checker = py_trees.composites.Selector(
            "BatteryChecker",
            memory=False,
            children=[
                py_trees.decorators.Inverter(
                    "IsBatteryLowInverter", IsBatteryLow("IsBatteryLow")
                ),
                py_trees.decorators.Inverter(
                    "SitActionInverter", SitAction("SitAction", self.node)
                ),
            ],
        )

        # 3) Hand‐gestures plugin + Spot interpreter
        plugins = py_trees.composites.Sequence(
            "Plugins",
            children=[
                PluginClient("HandGesturesPlugin", "landmark_detector_node", self.node),
                SpotGesturesInterpreterAction("SpotGesturesInterpreter", self.node),
            ],
        )

        self.add_children([robot_connection, battery_checker, plugins])


def bootstrap(ros_node: Node) -> py_trees.behaviour.Behaviour:
    return SpotBT(ros_node)
