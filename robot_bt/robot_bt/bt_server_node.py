import time
import py_trees
from py_trees.trees import BehaviourTree
from rclpy import Optional
from rclpy.logging import rclpy
from rclpy.node import Node
from robot_bt.bootstrap import bootstrap_bt


class BtServerNode(Node):
    _bt_tick_freq: int = 60  # Hz
    _bt_name: str = "spot_bt"
    _stop_on_failure: bool = False
    bt: Optional[BehaviourTree]

    def __init__(self, node_name):
        super().__init__(node_name)

        self.read_params()

        try:
            self.get_logger().info(f"Trying to bootstrap {self._bt_name} BT")
            bootstrap_fn = bootstrap_bt(self._bt_name)
            self.get_logger().info(f"{self._bt_name} has been bootstrapped")
            self.bt = BehaviourTree(root=bootstrap_fn(self))
            self.bt.setup()
        except:
            self.get_logger().error(f"Unable to bootstrap BT -> {self._bt_name}")
            self.bt = None

    def read_params(self):
        self.declare_parameter("bt_name", self._bt_name)
        self.declare_parameter("tick_freq", self._bt_tick_freq)
        self.declare_parameter("stop_on_failure", self._stop_on_failure)
        self.declare_parameter("plugins", [])

        self._bt_name = self.get_parameter("bt_name").get_parameter_value().string_value
        self._bt_tick_freq = (
            self.get_parameter("tick_freq").get_parameter_value().integer_value
        )
        self._stop_on_failure = (
            self.get_parameter("stop_on_failure").get_parameter_value().bool_value
        )

    def print_tree(self, tree: BehaviourTree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))

    def run(self):
        while rclpy.ok():
            if self.bt is None:
                self.get_logger().warning(
                    "Behaviour Tree was not successfully bootstrapped. No BT to run!"
                )
                time.sleep(2)
                continue
            self.bt.tick(post_tick_handler=self.print_tree)
            if (
                self.bt.root.status == py_trees.common.Status.FAILURE
                and self._stop_on_failure
            ):
                break
            rclpy.spin_once(self, timeout_sec=1)
            time.sleep(1 / self._bt_tick_freq)

        self.get_logger().warning("BT root has returned FAILURE. Shutting down")

        if self.bt:
            self.bt.shutdown()


def main():
    rclpy.init()
    node = BtServerNode("bt_server")

    node.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
