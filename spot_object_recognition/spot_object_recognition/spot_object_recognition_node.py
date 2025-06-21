import os
import csv
import threading

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import numpy as np
import cv2

from ultralytics import YOLO


class SpotObjectRecognitionNode(Node):

    def __init__(self):
        super().__init__('object_recognition_node')

        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('model_path', os.path.expanduser('~/Documents/spot/yolov8n.pt'))
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('detection_skip', 10)
        self.declare_parameter('csv_path', os.path.expanduser('~/Documents/spot/robot_suite_v2/detected_objects.csv'))
        self.declare_parameter('annotated_topic', '/camera/annotated_image')

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        model_path = os.path.expanduser(
            self.get_parameter('model_path').get_parameter_value().string_value
        )
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.detection_skip = self.get_parameter('detection_skip').get_parameter_value().integer_value
        csv_path_raw = self.get_parameter('csv_path').get_parameter_value().string_value
        self.csv_path = os.path.expanduser(csv_path_raw)
        annotated_topic = self.get_parameter('annotated_topic').get_parameter_value().string_value

        if not os.path.exists(model_path):
            self.get_logger().error(f'YOLO model file not found: {model_path}')
            raise FileNotFoundError(f'Expected YOLOv8 weights at {model_path}')

        self.get_logger().info(f'Loading YOLOv8 model (CPU) from: {model_path}')
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        self.unique_labels = set()

        if os.path.isfile(self.csv_path):
            try:
                with open(self.csv_path, 'r', newline='') as csvfile:
                    reader = csv.reader(csvfile)
                    rows = list(reader)
                    if rows and rows[0] == ['object']:
                        rows = rows[1:]
                    for row in rows:
                        if row:
                            self.unique_labels.add(row[0].strip())
                self.get_logger().info(f"Loaded {len(self.unique_labels)} existing labels from CSV.")
            except Exception as e:
                self.get_logger().warn(f"Failed to read existing CSV ({self.csv_path}): {e}")
        else:
            try:
                with open(self.csv_path, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['object'])
                self.get_logger().info(f"Created new CSV at {self.csv_path}")
            except Exception as e:
                self.get_logger().error(f"Cannot create CSV file at {self.csv_path}: {e}")
                raise

        self.image_count = 0

        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        self.annotated_publisher = self.create_publisher(Image, annotated_topic, 10)

        self.get_logger().info('ObjectRecognitionNode initialized, waiting for images...')

        self.csv_lock = threading.Lock()

    def image_callback(self, msg: Image):
        self.image_count += 1
        if self.image_count % self.detection_skip != 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        try:
            results = self.model(cv_image)[0]
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        for box, conf_tensor, cls_tensor in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
            confidence = float(conf_tensor.cpu().numpy())
            if confidence < self.conf_thresh:
                continue

            class_idx = int(cls_tensor.cpu().numpy())
            object = self.model.names[class_idx] if class_idx < len(self.model.names) else f"class_{class_idx}"

            x1, y1, x2, y2 = box.cpu().numpy().astype(int)

            cv2.rectangle(cv_image, (x1, y1), (x2, y2), color=(0, 255, 0), thickness=2)

            text = f"{object} {confidence * 100:.1f}%"

            (text_width, text_height), baseline = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, thickness=1
            )

            cv2.rectangle(
                cv_image,
                (x1, y1 - text_height - baseline - 4),
                (x1 + text_width + 2, y1),
                color=(0, 255, 0),
                thickness=-1
            )

            cv2.putText(
                cv_image,
                text,
                (x1 + 1, y1 - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                thickness=1
            )

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.annotated_publisher.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

        new_labels = []
        for box, conf_tensor, cls_tensor in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
            confidence = float(conf_tensor.cpu().numpy())
            if confidence < self.conf_thresh:
                continue

            class_idx = int(cls_tensor.cpu().numpy())
            object = self.model.names[class_idx] if class_idx < len(self.model.names) else f"class_{class_idx}"

            if object not in self.unique_labels:
                new_labels.append(object)

        if new_labels:
            with self.csv_lock:
                try:
                    with open(self.csv_path, 'a', newline='') as csvfile:
                        writer = csv.writer(csvfile)
                        for lbl in new_labels:
                            writer.writerow([lbl])
                            self.unique_labels.add(lbl)
                            self.get_logger().info(f"New object detected & logged: '{lbl}'")
                except Exception as e:
                    self.get_logger().error(f"Failed to write to CSV {self.csv_path}: {e}")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpotObjectRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down object recognition node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
