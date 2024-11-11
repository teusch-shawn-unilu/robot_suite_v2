import py_trees
import typing
from rclpy.node import Node
from tello_bt.nodes.actions.remote_operator import RemoteOperator
from tello_bt.nodes.conditionals.is_battery_low import IsBatteryLow
from tello_bt.nodes import PluginClient

from tello_bt.nodes.conditionals.can_run_plugin import CanRunPlugin


class DefaultBT(py_trees.composites.Sequence):
    def __init__(
        self,
        node: Node,
    ):
        super().__init__("DefaultBT", memory=True)
        self.node = node

        self.build_tree()

    def setup(self):  # type: ignore
        self.plugins_blackboard = py_trees.blackboard.Client(name="PluginsBlackboard")
        self.plugins_blackboard.register_key(
            "selected_plugin", access=py_trees.common.Access.WRITE
        )

        self.plugins_blackboard.selected_plugin = ""

    def build_tree(self):
        check_battery = py_trees.decorators.Inverter(
            "Check Battery",
            child=IsBatteryLow("IsBatteryLow", self.node),
        )

        remote_operator = RemoteOperator("RemoteOperator", self.node)

        run_hand_gestures = CanRunPlugin("CanRunHandGestures", "landmark_detector_node")
        hand_gestures_plugin = PluginClient(
            "HandGesturesPlugin", "landmark_detector_node", self.node
        )

        hand_gestures = py_trees.composites.Sequence(
            "HandGestures", False, children=[run_hand_gestures, hand_gestures_plugin]
        )

        self.add_children([check_battery, remote_operator, hand_gestures])
