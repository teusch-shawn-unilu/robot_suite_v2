from typing import Optional
from rclpy.node import Publisher, Subscription
from geometry_msgs.msg import Twist
import py_trees
from hand_gestures_msgs.msg import Landmarks
from robot_bt.behaviours.shared.actions import Action


class SpotGesturesInterpreterAction(Action):
    """
    Interpret hand‐gesture landmarks into Spot velocity commands.
    Stub implementation — fill in your own mapping logic.
    """
    cmd_vel_topic: str = "/cmd_vel"
    cmd_vel_pub: Publisher

    def setup(self) -> None:
        # subscribe to the hand‐landmarks topic (published by the hand_gestures_plugin)
        self.landmark_sub: Subscription = self.node.create_subscription(
            Landmarks,
            "/hand/landmarks",
            self._landmark_callback,
            1,
        )
        self.cmd_vel_pub = self.node.create_publisher(Twist, self.cmd_vel_topic, 1)
        self._latest: Optional[Landmarks] = None

    def _landmark_callback(self, msg: Landmarks) -> None:
        self._latest = msg

    def update(self) -> py_trees.common.Status:
        if self._latest is None:
            return py_trees.common.Status.FAILURE

        # TODO: translate self._latest into a Twist for Spot
        twist = Twist()
        # e.g., twist.linear.x = ...
        #       twist.angular.z = ...
        self.cmd_vel_pub.publish(twist)
        return py_trees.common.Status.SUCCESS
