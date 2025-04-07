import py_trees
from plugin_services.srv import PluginServer
from rclpy.node import Node
from rclpy.client import Client
from rclpy.logging import rclpy

STATUS_MAP = ["FAILURE", "RUNNING", "SUCCESS"]


class PluginClient(py_trees.behaviour.Behaviour):
    """Behaviour which uses ros2 services to control a plugin

    Use this class in case you want to tick a plugin which inherits
    the PluginBase class
    """

    client: Client

    def __init__(self, name: str, plugin_name: str, bt_node: Node):
        super().__init__(name)
        self.plugin_name = plugin_name
        self.node = bt_node

    def setup(self) -> None:  # type: ignore
        self.client = self.node.create_client(
            PluginServer, f"{self.plugin_name}/bt_server"
        )

    def _send_tick(self) -> PluginServer.Response | None:
        """Requests plugin to be ticked"""
        if not self.client.service_is_ready():
            self.node.get_logger().warning(
                f"Service {self.plugin_name}/bt_server is not ready. Not ticking."
            )
            return None

        request = PluginServer.Request()

        # TODO: Send the blackboard information if set
        request.blackboard = "{}"

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()

    def update(self) -> py_trees.common.Status:
        if self.client is None:
            self.node.get_logger().error("Make sure you have called setup method")
            return py_trees.common.Status.INVALID

        response = self._send_tick()

        if response is None:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status(STATUS_MAP[response.status])
