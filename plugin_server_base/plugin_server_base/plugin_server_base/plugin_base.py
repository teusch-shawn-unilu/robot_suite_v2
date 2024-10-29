from sys import exec_prefix
from rclpy.node import Node
from rclpy.service import SrvTypeResponse, SrvTypeRequest
from plugin_services.srv import PluginServer
import rclpy
from enum import Enum


class NodeState(Enum):
    FAILURE = 0
    RUNNING = 1
    SUCCESS = 2


class PluginBase(Node):
    tick_rate: int = 30
    standalone: bool = False

    def __init__(self, node_name: str) -> None:
        """Constructor of the class PluginBase.

        Args:
            - node_name(str): name for the ROS node
        """
        super().__init__(node_name)
        self.plugin_name = node_name

        self._read_base_params()

        if self.standalone:
            self.tick_timer = self.create_timer(1 / self.tick_rate, self._tick_callback)

        # Action server object which will control when the plugin runs
        self._server = self.create_service(
            PluginServer, f"{node_name}/bt_server", self._service_callback
        )

    def _read_base_params(self) -> None:
        self.declare_parameter("tick_rate", self.tick_rate)
        self.declare_parameter("standalone", self.standalone)

        self.tick_rate = (
            self.get_parameter("tick_rate").get_parameter_value().integer_value
        )
        self.standalone = (
            self.get_parameter("standalone").get_parameter_value().bool_value
        )

    def _tick_callback(self) -> None:
        """Callback method for the tick_timer."""
        try:
            self.tick()
        except Exception as e:
            self.get_logger().error(f"Tick Method has failed to run with {e}")

    def _service_callback(
        self, request: PluginServer, response: PluginServer
    ) -> PluginServer:
        """Sets the current state of the node based on the action request
        This will allow the node to be controlled by the BT_server
        This method should not be modified
        """

        # Send the current state before setting new state
        # This way, if the node fails, the BT will now and start its fallback
        # procedure

        self.get_logger().debug(
            f"Plugin <{self.plugin_name}> has been requested to be ticked with {request}"
        )

        node_state = self.tick()

        self.get_logger().debug(f"Plugin <{self.plugin_name}> state is: {node_state}")

        response.node_state = node_state.value

        return response

    def tick(self) -> NodeState:
        """This method should be implemented!
        You should place here what you want your node to do.
        It gets called `tick_rate` times a second if state=RUNNING
        """
        assert False, "Please make sure to implement the tick method of you Node"


def main():
    rclpy.init()
    node = PluginBase("test_plugin_node")

    def tick_dummy():
        print("Ticking...")
        return NodeState.SUCCESS

    node.tick = tick_dummy
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
