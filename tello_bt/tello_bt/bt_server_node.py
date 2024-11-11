import time
import py_trees
from py_trees.trees import BehaviourTree
from rclpy.logging import rclpy
from rclpy.node import Node
from tello_bt.bt.default_bt import DefaultBT


class BtServerNode(Node):
    bt_tick_freq: int = 60  # Hz

    def __init__(self, node_name):
        super().__init__(node_name)

        self.bt = BehaviourTree(root=DefaultBT(node=self))
        self.bt.setup()

    def read_params(self):
        pass

    def print_tree(self, tree: BehaviourTree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))

    def run(self):
        while rclpy.ok():
            self.bt.tick(post_tick_handler=self.print_tree)
            rclpy.spin_once(self, timeout_sec=1)
            time.sleep(1 / self.bt_tick_freq)

        self.bt.shutdown()


def main():
    rclpy.init()
    node = BtServerNode("bt_server")

    node.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
