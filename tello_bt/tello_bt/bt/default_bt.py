"""Default BT which can be used as an example.

This BT first checks the the drone is on the same network,
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

import py_trees
from rclpy.node import Node
from tello_bt.nodes.actions.land_action import LandAction
from tello_bt.nodes.actions.remote_operator import RemoteOperator
from tello_bt.nodes.conditionals.is_battery_low import IsBatteryLow
from tello_bt.nodes.conditionals.is_drone_connected import IsDroneConnected
from tello_bt.nodes import PluginClient

from tello_bt.nodes.conditionals.can_run_plugin import CanRunPlugin


class DefaultBT(py_trees.composites.Sequence):
    def __init__(
        self,
        node: Node,
    ):
        super().__init__("DefaultBT", memory=False)
        self.node = node

        self.build_tree()

    def setup(self):  # type: ignore
        self.plugins_blackboard = py_trees.blackboard.Client(name="PluginsBlackboard")
        self.plugins_blackboard.register_key(
            "selected_plugin", access=py_trees.common.Access.WRITE
        )

        self.plugins_blackboard.selected_plugin = ""

    def build_tree(self):
        drone_connection = py_trees.composites.Selector(
            "DroneConnection",
            memory=False,
            children=[
                IsDroneConnected("IsDroneConnected"),
            ],
        )

        battery_checker = py_trees.composites.Selector(
            "BatteryChecker",
            memory=False,
            children=[
                IsBatteryLow("IsBatteryLow", self.node),
                py_trees.decorators.Inverter(
                    "LandActionInverter", LandAction("LandAction", self.node)
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
                    memory=True,
                    children=[
                        CanRunPlugin("CanRunHandGestures", "landmark_detector_node"),
                        PluginClient(
                            "HandGesturesPlugin", "landmark_detector_node", self.node
                        ),
                    ],
                )
            ],
        )

        self.add_children([drone_connection, battery_checker, remote_operator, plugins])
