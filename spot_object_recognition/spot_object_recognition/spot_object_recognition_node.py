#!/usr/bin/env python3
import os
import csv
import threading

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import numpy as np
import cv2

# YOLOv8 import (CPU if no CUDA)
from ultralytics import YOLO


class SpotObjectRecognitionNode(Node):
    """
    Subscribes to /camera/image_raw, runs YOLOv8 (CPU), writes each unique object object
    to a CSV (no duplicates), and republishes an annotated image topic with bounding boxes
    and confidence percentages.
    """

    def __init__(self):
        super().__init__('object_recognition_node')

        # --- Declare & read parameters ---
        # Topic to subscribe for images
        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')
        # Path to YOLOv8 weights (.pt file)
        self.declare_parameter('model_path', os.path.expanduser('~/Documents/spot/yolov8n.pt'))
        # Minimum confidence threshold (0.0–1.0)
        self.declare_parameter('confidence_threshold', 0.5)
        # How many frames to skip between inferences (to lighten CPU load)
        self.declare_parameter('detection_skip', 10)
        # CSV file path (will store unique labels, one per line)
        self.declare_parameter('csv_path', os.path.expanduser('~/Documents/spot/robot_suite_v2/detected_objects.csv'))
        # Topic to publish annotated images
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

        # --- Load YOLOv8 model (CPU) ---
        if not os.path.exists(model_path):
            self.get_logger().error(f'YOLO model file not found: {model_path}')
            raise FileNotFoundError(f'Expected YOLOv8 weights at {model_path}')

        self.get_logger().info(f'Loading YOLOv8 model (CPU) from: {model_path}')
        self.model = YOLO(model_path)  # device='cpu' force CPU inference

        # Prepare CvBridge
        self.bridge = CvBridge()

        # Maintain a set of all labels ever logged
        self.unique_labels = set()

        # If CSV already exists, read its contents to seed unique_labels
        if os.path.isfile(self.csv_path):
            try:
                with open(self.csv_path, 'r', newline='') as csvfile:
                    reader = csv.reader(csvfile)
                    # If there's a header, skip it
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
            # If file doesn't exist, create it and write header
            try:
                with open(self.csv_path, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['object'])
                self.get_logger().info(f"Created new CSV at {self.csv_path}")
            except Exception as e:
                self.get_logger().error(f"Cannot create CSV file at {self.csv_path}: {e}")
                raise

        # Frame counter for throttling
        self.image_count = 0

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        # Publisher for annotated images
        self.annotated_publisher = self.create_publisher(Image, annotated_topic, 10)

        self.get_logger().info('ObjectRecognitionNode initialized, waiting for images...')

        # Lock to prevent concurrent writes to CSV
        self.csv_lock = threading.Lock()

    def image_callback(self, msg: Image):
        # Throttle inference: skip some frames
        self.image_count += 1
        if self.image_count % self.detection_skip != 0:
            return

        # Convert ROS Image → CV2 BGR image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # Run YOLOv8 inference (returns a list of results; take [0] since single image)
        try:
            results = self.model(cv_image)[0]
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        # Prepare to draw bounding boxes
        # Results: .boxes.xyxy (coordinates), .boxes.conf (confidence), .boxes.cls (class index)
        for box, conf_tensor, cls_tensor in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
            confidence = float(conf_tensor.cpu().numpy())
            if confidence < self.conf_thresh:
                continue

            class_idx = int(cls_tensor.cpu().numpy())
            object = self.model.names[class_idx] if class_idx < len(self.model.names) else f"class_{class_idx}"

            # Extract bounding box coordinates
            x1, y1, x2, y2 = box.cpu().numpy().astype(int)

            # Draw rectangle
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), color=(0, 255, 0), thickness=2)

            # Prepare object text with confidence percentage
            text = f"{object} {confidence * 100:.1f}%"

            # Determine text size & baseline
            (text_width, text_height), baseline = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, thickness=1
            )

            # Draw filled rectangle behind text for readability
            cv2.rectangle(
                cv_image,
                (x1, y1 - text_height - baseline - 4),
                (x1 + text_width + 2, y1),
                color=(0, 255, 0),
                thickness=-1
            )

            # Put text (object + confidence) above the box
            cv2.putText(
                cv_image,
                text,
                (x1 + 1, y1 - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                thickness=1
            )

        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.annotated_publisher.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

        # For each detection: check confidence & object, and log new labels to CSV
        new_labels = []
        for box, conf_tensor, cls_tensor in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
            confidence = float(conf_tensor.cpu().numpy())
            if confidence < self.conf_thresh:
                continue

            class_idx = int(cls_tensor.cpu().numpy())
            object = self.model.names[class_idx] if class_idx < len(self.model.names) else f"class_{class_idx}"

            # If this object is new, add to new_labels
            if object not in self.unique_labels:
                new_labels.append(object)

        # If any brand-new labels found, append them to CSV
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
        # On shutdown, nothing special; the CSV is already closed by context managers
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
