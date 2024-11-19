import time
import py_trees
from py_trees.trees import BehaviourTree
from rclpy.logging import rclpy
from rclpy.node import Node
from tello_bt.bt.default_bt import DefaultBT, bootstrap
from tello_bt.bootstrap import bootstrap_bt, BootstrapError


class BtServerNode(Node):
    _bt_tick_freq: int = 60  # Hz
    _bt_name: str = "default_bt"

    def __init__(self, node_name):
        super().__init__(node_name)

        self.read_params()

        try:
            bootstrap_fn = bootstrap_bt(self._bt_name)
        except (ImportError, BootstrapError) as e:
            raise Exception("Failed to bootstrap BT")

        if bootstrap_fn is None:
            raise Exception("Ensure that your bootstrap function returns a tree")

        self.bt = BehaviourTree(root=bootstrap_fn(self))
        self.bt.setup()

    def read_params(self):
        self.declare_parameter("bt_name", self._bt_name)
        self.declare_parameter("tick_freq", self._bt_tick_freq)

        self._bt_name = self.get_parameter("bt_name").get_parameter_value().string_value
        self._bt_tick_freq = (
            self.get_parameter("tick_freq").get_parameter_value().integer_value
        )

    def print_tree(self, tree: BehaviourTree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))

    def run(self):
        while rclpy.ok():
            self.bt.tick(post_tick_handler=self.print_tree)
            rclpy.spin_once(self, timeout_sec=1)
            time.sleep(1 / self._bt_tick_freq)

        self.bt.shutdown()


def main():
    rclpy.init()
    node = BtServerNode("bt_server")

    node.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
