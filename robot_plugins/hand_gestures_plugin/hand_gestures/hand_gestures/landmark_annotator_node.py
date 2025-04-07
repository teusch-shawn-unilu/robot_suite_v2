from __future__ import annotations
import rclpy
import cv_bridge
import hand_gestures.helpers as helpers
from message_filters import ApproximateTimeSynchronizer, Subscriber

from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.node import Node
from hand_gestures_msgs.msg import Landmarks
from sensor_msgs.msg import Image
from ament_index_python import get_package_share_directory


class LandmarkAnnotatorNode(Node):
    """Node to detect hand landmakrs from an input image

    Subscriptions:
        - /camera/image_raw (sensor_msgs/Image): Input image
        - /hand/landmakrs (hand_gestures_msgs/Landmarks): Input landmarks

    Publications:
        - /hand/landmarks (hand_gestures_msgs/Landmarks): Detected hand landmarks

    Parameters:
        - img_input_topic (str): Input image topic [default: /camera/image_raw]
        - landmarks_topic (str): Output landmarks topic [default: /hand/landmarks]
        - annotated_img_topic (str): Output annotated image topic [default: /hand/annotated/image]
    """

    img_input_topic: str = "/camera/image_raw"
    annotated_img_topic: str = "/hand/annotated/image"
    landmarks_input_topic: str = "/hand/landmarks"
    input_img_sub_backup: Subscription
    input_img_sub: Subscriber
    landmarks_sub: Subscriber
    annotated_img_pub: Publisher

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.load_parameters()

        # This subscription is used to get the input image in case no landmarks are received
        self.input_img_sub_backup = self.create_subscription(
            Image, self.img_input_topic, self.img_input_callback, 10
        )

        self.input_img_sub = Subscriber(self, Image, self.img_input_topic)

        self.annotated_img_pub = self.create_publisher(
            Image, self.annotated_img_topic, 10
        )

        self.landmarks_sub = Subscriber(self, Landmarks, self.landmarks_input_topic)

        # Synchronizing the input image and landmarks
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.input_img_sub, self.landmarks_sub], 10, 0.1
        )
        self.synchronizer.registerCallback(self.callback)

        self.pkg_share_dir = get_package_share_directory("hand_gestures")
        self.bridge = cv_bridge.CvBridge()
        self.last_received_time = 0

    def load_parameters(self) -> None:
        self.declare_parameter("img_input_topic", self.img_input_topic)
        self.declare_parameter("landmarks_input_topic", self.landmarks_input_topic)
        self.declare_parameter("annotated_img_topic", self.annotated_img_topic)

        self.img_input_topic = (
            self.get_parameter("img_input_topic").get_parameter_value().string_value
        )
        self.landmarks_input_topic = (
            self.get_parameter("landmarks_input_topic")
            .get_parameter_value()
            .string_value
        )
        self.annotated_img_topic = (
            self.get_parameter("annotated_img_topic").get_parameter_value().string_value
        )

    def img_input_callback(self, msg: Image) -> None:
        self.get_logger().debug("Received image")
        if self.get_clock().now().nanoseconds - self.last_received_time > 1e8:
            self.annotated_img_pub.publish(msg)

    def callback(self, image_msg, landmarks_msg):
        received_image = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
        self.last_received_time = self.get_clock().now().nanoseconds
        self.__annotate_image(received_image, landmarks_msg)

    def __annotate_image(self, img, landmarks) -> None:
        annotated_img = helpers.annotate_landmarks(img, landmarks)

        self.annotated_img_pub.publish(self.bridge.cv2_to_imgmsg(annotated_img, "rgb8"))


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkAnnotatorNode("landmark_annotator_node")
    rclpy.spin(node)
    rclpy.shutdown()
