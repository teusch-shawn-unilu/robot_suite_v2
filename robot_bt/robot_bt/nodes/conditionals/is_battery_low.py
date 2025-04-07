from robot_bt.nodes import Action
import py_trees

from sensor_msgs.msg import BatteryState


class IsBatteryLow(Action):
    battery_topic: str = "/battery_state"
    low_battery_threshold: float = 20

    def setup(self) -> None:  # type: ignore
        self.current_battery = -1
        self.battery_sub = self.node.create_subscription(
            BatteryState, self.battery_topic, self.battery_callback, 1
        )

    def battery_callback(self, msg: BatteryState):
        self.current_battery = msg.percentage

    def update(self) -> py_trees.common.Status:
        if self.current_battery < self.low_battery_threshold:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
