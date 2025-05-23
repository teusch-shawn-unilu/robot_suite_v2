import rclpy
from robot_bt.behaviours.shared.actions import Action
from bosdyn.client.robot_command import RobotCommandBuilder

class SitAction(Action):
    """
    Tell Spot to sit when invoked by the BT.
    """
    def __init__(self, node: rclpy.node.Node, name: str = "SitAction"):
        super().__init__(node, name)

        self._cmd_client = self._bosdyn_robot.ensure_client('robot-command')

    # ----------------------------------------------------------------
    def update(self) -> Action.Status:
        try:
            sit_cmd = RobotCommandBuilder.synchro_sit_command()
            self._cmd_client.robot_command(sit_cmd)
            return Action.Status.SUCCESS
        except Exception as exc:
            self.node.get_logger().error(f"Sit failed: {exc}")
            return Action.Status.FAILURE