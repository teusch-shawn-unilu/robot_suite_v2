import py_trees
from plugin_services.srv import PluginServer
from rclpy.node import Node
from rclpy.client import Client
from rclpy.logging import rclpy

STATUS_MAP = ["FAILURE", "RUNNING", "SUCCESS"]


class Behavior(py_trees.behaviour.Behaviour):
    client: Client | None = None

    def __init__(self, name, bt_node: Node):
        super().__init__(name)
        self.node = bt_node

    def setup(self) -> None:  # type: ignore
        self.client = self.node.create_client(PluginServer, f"{self.name}/bt_server")

        # TODO: Perhaps add a maximum number of tries to reach the service
        # Otherwise, this will wait here in an infinite state

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f"Waiting for service {self.name}/bt_server becomes available"
            )

    def _send_tick(self) -> PluginServer.Response | None:
        if self.client is None:
            self.node.get_logger().error(f"Service client has not been created")
            return None
        request = PluginServer.Request()
        request.tick = True
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()

    def update(self) -> py_trees.common.Status:
        response = self._send_tick()
        if response is None:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status(STATUS_MAP[response.node_state])
