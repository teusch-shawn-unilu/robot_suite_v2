from robot_bt.behaviours.shared.actions import PluginClient
from robot_bt.behaviours.shared.conditions import CanRunPlugin, IsBatteryLow
from robot_bt.behaviours.spot.conditions import IsRobotConnected

import py_trees
from rclpy.node import Node
from robot_bt.behaviours.spot.actions import SitAction
from robot_bt.behaviours.spot.actions import RemoteOperator
from robot_bt.behaviours.spot.actions import SpotGesturesInterpreterAction

"""Default BT which can be used as an example.

This BT first checks the the robot is on the same network,
then checks if it has enough battery, it has a remote remote_operator
to toggle which plugin to run and finally the hand_gestures plugin.

This is how the tree looks like:

[-] DefaultBT [✕]
    [o] DroneConnection [✕]
        --> IsDroneConnected [✕]
    [o] BatteryChecker [-]
        --> IsBatteryLow [-]
        -^- LandActionInverter [-]
            --> LandAction [-]
    --> RemoteOperator [-]
    [o] Plugins [-]
        {-} HandGesturesControl [-]
            --> CanRunHandGestures [-]
            --> HandGesturesPlugin [-]
"""

class SpotBT(py_trees.composites.Sequence):
    def __init__(self, node: Node,):
        super().__init__("SpotBT", memory=False)
        self.node = node
        self.build_tree()

    def setup(self):  # type: ignore
        self.plugins_blackboard = py_trees.blackboard.Client(name="PluginsBlackboard")
        self.plugins_blackboard.register_key("selected_plugin", access=py_trees.common.Access.WRITE)
        self.plugins_blackboard.selected_plugin = ""

    def build_tree(self):
        robot_connection = py_trees.composites.Selector(
            "RobotConnection",
            memory=False,
            children=[
                IsRobotConnected("IsRobotConnected"),
            ],
        )

        battery_checker = py_trees.composites.Selector(
            "BatteryChecker",
            memory=False,
            children=[
                py_trees.decorators.Inverter(
                    "IsBAtteryLowInverter", IsBatteryLow("IsBatteryLow", self.node)
                ),
                py_trees.decorators.Inverter(
                    "SitActionInverter", SitAction("SitAction", self.node)
                ),
            ],
        )

        remote_operator = RemoteOperator("RemoteOperator", self.node)

        plugins = py_trees.composites.Selector(
            "Plugins",
            memory=False,
            children=[
                py_trees.composites.Sequence(
                    "HandGesturesControl",
                    memory=False,
                    children=[
                        CanRunPlugin("CanRunHandGestures", "landmark_detector_node"),
                        PluginClient(
                            "HandGesturesPlugin", "landmark_detector_node", self.node
                        ),
                        SpotGesturesInterpreterAction(
                            "SpotGesturesInterpreterAction", self.node
                        ),
                    ],
                )
            ],
        )

        #, battery_checker
        self.add_children([robot_connection, remote_operator, plugins])


def bootstrap(ros_node: Node) -> py_trees.behaviour.Behaviour:
    return SpotBT(ros_node)


