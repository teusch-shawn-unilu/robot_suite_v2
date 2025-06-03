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
    Subscribes to /camera/image_raw, runs YOLOv8 (CPU), and writes each unique object label
    to a CSV (no duplicates). If the CSV already exists, reads existing labels at startup to avoid re‐logging.
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

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        model_path = os.path.expanduser(
            self.get_parameter('model_path').get_parameter_value().string_value
        )
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.detection_skip = self.get_parameter('detection_skip').get_parameter_value().integer_value
        csv_path_raw = self.get_parameter('csv_path').get_parameter_value().string_value
        self.csv_path = os.path.expanduser(csv_path_raw)

        # --- Load YOLOv8 model (CPU) ---
        if not os.path.exists(model_path):
            self.get_logger().error(f'YOLO model file not found: {model_path}')
            raise FileNotFoundError(f'Expected YOLOv8 weights at {model_path}')

        self.get_logger().info(f'Loading YOLOv8 model (CPU) from: {model_path}')
        self.model = YOLO(model_path)  # device='cpu' force CPU inference

        # Prepare CvBridge
        self.bridge = CvBridge()
        self.camera = cv2.VideoCapture(0)

        # Maintain a set of all labels ever logged
        self.unique_labels = set()

        # If CSV already exists, read its contents to seed unique_labels
        if os.path.isfile(self.csv_path):
            try:
                with open(self.csv_path, 'r', newline='') as csvfile:
                    reader = csv.reader(csvfile)
                    # If there's a header, skip it
                    rows = list(reader)
                    if rows and rows[0] == ['label']:
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
                    writer.writerow(['label'])
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

        # Run YOLOv8 inference (returns a list of results; we take [0] since single image)
        results = None
        try:
            results = self.model(cv_image)[0]
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        # For each detection: check confidence & label
        # YOLOv8 results.boxes: .xyxy, .conf, .cls
        new_labels = []
        for box, conf_tensor, cls_tensor in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
            confidence = float(conf_tensor.cpu().numpy())
            if confidence < self.conf_thresh:
                continue

            class_idx = int(cls_tensor.cpu().numpy())
            label = self.model.names[class_idx] if class_idx < len(self.model.names) else f"class_{class_idx}"

            # If this label is new, add to new_labels
            if label not in self.unique_labels:
                new_labels.append(label)

        # If any brand-new labels found, append them to CSV
        if new_labels:
            # Acquire lock before writing
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