import rclpy
import unittest
from plugin_server_base.plugin_base import PluginBase, NodeState
from plugin_services.srv import PluginServer
from rclpy.client import Client

PLUGIN_NAME = "test_plugin_base"


class TestPluginBase(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()

        self.node = PluginBase(PLUGIN_NAME)

        # Override the tick method for testing
        def tick_dummy(blackboard=None):
            self.node.get_logger().info("Tick method called.")
            return NodeState.SUCCESS

        self.node.tick = tick_dummy

    def tearDown(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()

    def test_bt_integration(self):
        """Test that tick method is called when the service is called"""
        client = self.node.create_client(PluginServer, f"{PLUGIN_NAME}/bt_server")

        assert client.service_is_ready(), "Service Should have been ready"

        request = PluginServer.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        result = future.result()
        assert result is not None

        assert result.status == 2

    def test_standalone_mode(self):
        """Test that tick method is ticked when in standalone mode"""

        self.node.standalone = True
        self.node.tick_rate = 10  # 10 Hz
        self.node.tick_timer = self.node.create_timer(
            1 / self.node.tick_rate, self.node._tick_callback
        )

        # Set a counter to track tick calls
        tick_counter = {"count": 0}

        def tick_counting_dummy(blackboard=None):
            tick_counter["count"] += 1
            return NodeState.SUCCESS

        self.node.tick = tick_counting_dummy
        rclpy.spin_once(self.node, timeout_sec=0.1)

        assert tick_counter["count"] == 1


if __name__ == "__main__":
    unittest.main()
