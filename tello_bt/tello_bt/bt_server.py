from rclpy.logging import rclpy
from rclpy.node import Node
from tello_bt.behavior import Behavior


class BtServerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        behavior = Behavior("test_plugin_node", self)
        behavior.setup()
        behavior.update()


def main():
    rclpy.init()
    node = BtServerNode("bt_server")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
