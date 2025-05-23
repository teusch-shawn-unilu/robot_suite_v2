from typing import Optional

from rclpy.node import Node
from sensor_msgs.msg import Image
from synchros2.utilities import namespace_with
from cv_bridge import CvBridge
from synchros2.tf_listener_wrapper import TFListenerWrapper
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from spot_msgs.action import RobotCommand
from std_srvs.srv import Trigger
from .simple_spot_commander import SimpleSpotCommander

from hand_gestures_msgs.msg import Landmarks

from robot_bt.behaviours.shared.actions import Action
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import py_trees



class SpotGesturesInterpreterAction(Action):

    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        if self.node is None:
            raise ValueError("no ROS 2 node available.")
        
        self.logger = self.node.get_logger()

        self.odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
        self.grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)

        self.landmarks_sub = self.node.create_subscription(Landmarks, self.landmarks_topic, self.landmarks_callback, 10)
        self.cmd_pub = self.node.create_publisher(Twist, self.cmd_topic, 10)

        # Trigger commands for the Spot robot to do a preprogrammed action
        self.cmd_sit = self.node.create_client(Trigger, namespace_with(robot_name, "sit"))
        self.cmd_stand = self.node.create_client(Trigger, namespace_with(robot_name, "stand"))

        # Wrapper to accept Strings as commands for Spot. 
        # SimpleSpotCommander executes the commands while the ActionClientWrapper accepts String and 
        # transforms them into the correct command
        self.robot = SimpleSpotCommander(robot_name, node)
        self.robot_command_client = ActionClientWrapper(RobotCommand, namespace_with(robot_name, "robot_command"), node)


    # detects the number of hands present on the image and returns the number of hands
    def detected_hands(self, msg: Landmarks) -> int:
        result = 0
        if msg.left_hand.handedness == "":
            result += 1
        if msg.right_hand.handedness == "":
            result += 1
        
        return result
    
    def start(self) -> bool:
        # Claim robot
        self.logger.info("Claiming robot")
        result = self.robot.command("claim")
        if not result.success:
            self.logger.error("Unable to claim robot message was " + result.message)
            return False
        self.logger.info("Claimed robot")

        # Power on robot
        self.logger.info("Powering on robot")
        result = self.robot.command("power_on")
        if not result.success:
            self.logger.error("Unable to power on robot message was " + result.message)
            return False

        self.logger.info("Standing robot up")
        result = self.robot.command("stand")
        if not result.success:
            self.logger.error("Robot did not stand message was " + result.message)
            return False
        self.logger.info("Successfully stood up.")

        return True
    
    def stand(self, msg: Landmarks) -> py_trees.common.Status:
        # The command to stand up is made with only one hand
        # Return false if there is no hand or more than one hand
        if(self.detected_hands(msg) != 1):
            return py_trees.common.Status.FAILURE
        
        if msg.right_hand.gesture == "ILoveYou":    
            self.logger.info("Standing robot up")
            result = self.robot.command("stand")
        
            if not result.success:
                self.logger.error("Robot did not stand! Message was " + result.message)
                return py_trees.common.Status.FAILURE
            
            self.logger.info("Successfully stood up.")
        
        return py_trees.common.Status.SUCCESS
    
    def sit(self, msg: Landmarks) -> py_trees.common.Status:
        if self.detected_hands(msg) != 2:
            return py_trees.common.Status.FAILURE

        if msg.left_hand.gesture == "Open_Palm" and msg.right_hand.gesture == "Thumb_Down":
            self.logger.info("Sitting robot down")
            result = self.robot.command("sit")    
        else:
            self.logger.info("Robot did not sit down! Message was " + result.message)
            return py_trees.common.Status.FAILURE

        self.logger.info("Successfully sat down.")
        return py_trees.common.Status.SUCCESS
    
    def _sit(self) -> None:
        self.cmd_sit.call_async(Trigger.Request())

    def _stand(self) -> None:
        self.cmd_stand.call_async(Trigger.Request())

    def landmarks_callback(self, msg: Landmarks) -> None:
        self.last_landmark_recv = msg

    # def nbr_hands_detected(self, msg: Landmarks) -> int:
    #     return int(msg.right_hand.handedness != "") + int(
    #         msg.left_hand.handedness != ""
    #     )

    # def can_takeoff(self, msg: Landmarks) -> bool:
    #     if self.nbr_hands_detected(msg) != 1:
    #         return False

    #     if not msg.right_hand.gesture == "ILoveYou":
    #         return False

    #     return True

    # def can_land(self, msg: Landmarks) -> bool:
    #     if self.nbr_hands_detected(msg) != 2:
    #         return False

    #     if not (
    #         msg.left_hand.gesture == "Open_Palm"
    #         and msg.right_hand.gesture == "Thumb_Down"
    #     ):
    #         return False

    #     return True

    # def can_flip(self, msg: Landmarks) -> bool:
    #     if self.nbr_hands_detected(msg) != 2:
    #         return False

    #     if not (
    #         msg.left_hand.gesture == "ILoveYou" and msg.right_hand.gesture == "ILoveYou"
    #     ):
    #         return False

    #     return True

    # def build_move(self, msg: Landmarks) -> Twist | None:
    #     if self.nbr_hands_detected(msg) != 2:
    #         return None

    #     left_gesture, right_gesture = msg.left_hand.gesture, msg.right_hand.gesture

    #     cmd_vel = Twist()

    #     # Forward/backward
    #     if left_gesture == "Open_Palm":
    #         if right_gesture == "Pointing_Up":
    #             cmd_vel.linear.x = self.max_lin_speed
    #         elif right_gesture == "Victory":
    #             cmd_vel.linear.x = -self.max_lin_speed

    #     # Left/right
    #     if left_gesture == "Pointing_Up" and right_gesture == "Closed_Fist":
    #         cmd_vel.linear.y = -self.max_lin_speed
    #     elif left_gesture == "Closed_Fist" and right_gesture == "Pointing_Up":
    #         cmd_vel.linear.y = self.max_lin_speed

    #     # Up/Down
    #     if left_gesture == "Thumb_Up" and right_gesture == "Thumb_Up":
    #         cmd_vel.linear.z = self.max_lin_speed
    #     elif left_gesture == "Thumb_Down" and right_gesture == "Thumb_Down":
    #         cmd_vel.linear.z = -self.max_lin_speed

    #     # Rotation left/right
    #     if left_gesture == "Thumb_Up" and right_gesture == "Closed_Fist":
    #         cmd_vel.angular.z = -self.max_lin_speed
    #     elif left_gesture == "Closed_Fist" and right_gesture == "Thumb_Up":
    #         cmd_vel.angular.z = self.max_lin_speed

    #     return cmd_vel

    # def takeoff(self) -> None:
    #     self.takeoff_pub.publish(Empty())

    # def land(self) -> None:
    #     self.land_pub.publish(Empty())

    # def flip(self) -> None:
    #     msg = FlipControl()
    #     choice = random.choice(["left", "right", "back"])
    #     # Not using forward since it can be dangerous

    #     if choice == "left":
    #         msg.flip_left = True
    #     elif choice == "right":
    #         msg.flip_right = True
    #     elif choice == "forward":
    #         msg.flip_forward = True
    #     elif choice == "back":
    #         msg.flip_backward = True

    #     self.flip_pub.publish(msg)

    # def move(self, cmd: Twist) -> None:
    #     self.cmd_pub.publish(cmd)

    def update(self) -> py_trees.common.Status:
        if self.last_landmark_recv is None:
            return py_trees.common.Status.FAILURE

        landmark_msg = self.last_landmark_recv

        if self.can_takeoff(landmark_msg):
            self.takeoff()
            return py_trees.common.Status.SUCCESS

        if self.can_land(landmark_msg):
            self.land()
            return py_trees.common.Status.SUCCESS

        if self.can_flip(landmark_msg):
            self.flip()
            return py_trees.common.Status.SUCCESS

        mouvement = self.build_move(landmark_msg)
        if mouvement is None:
            return py_trees.common.Status.FAILURE

        self.move(mouvement)
        return py_trees.common.Status.SUCCESS
