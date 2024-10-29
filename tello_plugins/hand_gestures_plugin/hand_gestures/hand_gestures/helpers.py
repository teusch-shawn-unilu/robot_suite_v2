from __future__ import annotations
import os
import numpy as np

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"  # Suppress TensorFlow logging
from typing import Optional
from hand_gestures_msgs.msg import Landmarks
from geometry_msgs.msg import Point
from mediapipe.tasks.python.vision import (
    GestureRecognizerResult,
)
from mediapipe.tasks.python.components.containers import (
    NormalizedLandmark,
    Landmark,
)
from numpy.typing import NDArray
import numpy as np
import cv2

GREEN_COLOR = (47, 208, 68)
RED_COLOR = (248, 24, 7)
BLUE_COLOR = (0, 0, 255)


def __serialize_landmark(
    normalized_landmakrs: list[NormalizedLandmark],
    world_landmarks: list[Landmark],
) -> Optional[tuple[list[Point], list[Point]]]:
    """Serialize the landmarks to a ROS message.

    Args:
        normalized_landmakrs (list[NormalizedLandmark]): The normalized landmarks
        world_landmarks (list[Landmark]): The world Landmarks

    Returns:
        Optional[tuple[list[Point], list[Point]]]: The serialized landmarks
    """
    normalized_msg: list[Point] = []
    world_msg: list[Point] = []

    if (
        len(normalized_landmakrs) == 0
        or len(world_landmarks) == 0
        or len(normalized_landmakrs) != len(world_landmarks)
    ):
        return None

    for idx in range(len(normalized_landmakrs)):
        normalized_point = Point(
            x=normalized_landmakrs[idx].x,
            y=normalized_landmakrs[idx].y,
            z=normalized_landmakrs[idx].z,
        )
        world_point = Point(
            x=world_landmarks[idx].x,
            y=world_landmarks[idx].y,
            z=world_landmarks[idx].z,
        )
        normalized_msg.append(normalized_point)
        world_msg.append(world_point)

    return normalized_msg, world_msg


def serialize_gesture_recognizer_result(
    landmark_result: GestureRecognizerResult,
) -> Optional[Landmarks]:
    """Serialize the landmarks to a ROS message

    Args:
        landmark_result (HandLandmarkerResult): The landmarks to serialize

    Returns:
        Optional[Landmarks]: The serialized landmarks
    """
    handedness = landmark_result.handedness
    gestures = landmark_result.gestures
    normalized_landmarks = landmark_result.hand_landmarks
    world_landmarks = landmark_result.hand_world_landmarks

    if len(handedness) == 0:
        return None

    msg = Landmarks()

    for idx, (hand, gesture) in enumerate(zip(handedness, gestures)):
        hand = hand[0]
        gesture = gesture[0]
        if hand.category_name == "Right":
            msg.right_hand.handedness = hand.category_name
            msg.right_hand.gesture = gesture.category_name
            msg.right_hand.score = hand.score
            ret = __serialize_landmark(normalized_landmarks[idx], world_landmarks[idx])
            if ret is None:
                return None
            msg.right_hand.normalized_landmarks = ret[0]
            msg.right_hand.world_landmarks = ret[1]
        elif hand.category_name == "Left":
            msg.left_hand.handedness = hand.category_name
            msg.left_hand.gesture = gesture.category_name
            msg.left_hand.score = hand.score
            ret = __serialize_landmark(normalized_landmarks[idx], world_landmarks[idx])
            if ret is None:
                return None
            msg.left_hand.normalized_landmarks = ret[0]
            msg.left_hand.world_landmarks = ret[1]

    return msg


def _bounding_box_from_landmarks(
    img_width, img_height, landmarks: list[Landmark], margin: int = 10
) -> Optional[tuple[int, int, int, int]]:
    if len(landmarks) == 0:
        return None
    landmark_array = np.empty((0, 2), int)

    for landmark in landmarks:
        landmark_x = min(int(landmark.x * img_width), img_width - 1)
        landmark_y = min(int(landmark.y * img_height), img_height - 1)

        landmark_point = [np.array((landmark_x, landmark_y))]

        landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv2.boundingRect(landmark_array)

    x = max(x - margin, 0)
    y = max(y - margin, 0)
    w = min(w + 2 * margin, img_width - x)
    h = min(h + 2 * margin, img_height - y)
    return x, y, w, h


def _draw_hand_bounding_box(
    image: NDArray,
    landmarks: Landmarks,
):
    """Draw the bounding box around the hands

    Args:
        image (NDArray): The image to draw the bounding box
        landmarks (Landmarks): The landmarks to draw the bounding box

    Returns:
        NDArray: The image with the bounding box
    """
    height, width, _ = image.shape

    # Left
    ret = _bounding_box_from_landmarks(
        width, height, landmarks.left_hand.normalized_landmarks  # type: ignore
    )
    if ret is not None:
        x, y, w, h = ret
        cv2.rectangle(image, (x, y), (x + w, y + h), GREEN_COLOR, 2)

    # Right
    ret = _bounding_box_from_landmarks(
        width, height, landmarks.right_hand.normalized_landmarks  # type: ignore
    )
    if ret is not None:
        x, y, w, h = ret
        cv2.rectangle(image, (x, y), (x + w, y + h), GREEN_COLOR, 2)


def _draw_landmark_points(image: NDArray, landmarks: Landmarks):
    """Draw the landmarks on the image

    Args:
        image (NDArray): The image to draw the landmarks
        landmarks (Landmarks): The landmarks to draw
    """
    height, width, _ = image.shape

    for idx in range(21):
        if len(landmarks.left_hand.normalized_landmarks) > 0:
            landmark = landmarks.left_hand.normalized_landmarks[idx]  # type: ignore
            landmark_x = min(int(landmark.x * width), width - 1)
            landmark_y = min(int(landmark.y * height), height - 1)
            cv2.circle(
                image,
                (landmark_x, landmark_y),
                8,
                GREEN_COLOR,
                -1,
                cv2.LINE_AA,
            )
            cv2.circle(
                image,
                (landmark_x, landmark_y),
                5,
                RED_COLOR,
                -1,
                cv2.LINE_AA,
            )
        if len(landmarks.right_hand.normalized_landmarks) > 0:
            landmark = landmarks.right_hand.normalized_landmarks[idx]  # type: ignore
            landmark_x = min(int(landmark.x * width), width - 1)
            landmark_y = min(int(landmark.y * height), height - 1)
            cv2.circle(
                image,
                (landmark_x, landmark_y),
                8,
                RED_COLOR,
                -1,
                cv2.LINE_AA,
            )
            cv2.circle(
                image,
                (landmark_x, landmark_y),
                5,
                GREEN_COLOR,
                -1,
                cv2.LINE_AA,
            )


def _draw_connections(image: NDArray, landmarks: Landmarks):
    """Draw the connections between the landmarks

    Args:
        image (NDArray): The image to draw the connections
        landmarks (Landmarks): The landmarks to draw the connections
    """
    # fmt: off
    CONNECTIONS = [
        (0, 1), (1, 2), (2, 3), (3, 4), (5, 6), 
        (6, 7), (7, 8), (9, 10), (10, 11), (11, 12), 
        (13, 14), (14, 15), (15, 16), (17, 18),(18, 19),
        (19, 20), (0, 5), (5, 9), (9, 13), (13, 17), (0, 17),
    ]
    # fmt: on
    height, width, _ = image.shape

    for connection in CONNECTIONS:
        if len(landmarks.left_hand.normalized_landmarks) > 0:
            start_landmark = landmarks.left_hand.normalized_landmarks[  # type: ignore
                connection[0]
            ]
            end_landmark = landmarks.left_hand.normalized_landmarks[connection[1]]  # type: ignore
            start_point = (
                min(int(start_landmark.x * width), width - 1),
                min(int(start_landmark.y * height), height - 1),
            )
            end_point = (
                min(int(end_landmark.x * width), width - 1),
                min(int(end_landmark.y * height), height - 1),
            )
            cv2.line(image, start_point, end_point, GREEN_COLOR, 3, cv2.LINE_AA)

        if len(landmarks.right_hand.normalized_landmarks) > 0:
            start_landmark = landmarks.right_hand.normalized_landmarks[  # type: ignore
                connection[0]
            ]
            end_landmark = landmarks.right_hand.normalized_landmarks[connection[1]]  # type: ignore
            start_point = (
                min(int(start_landmark.x * width), width - 1),
                min(int(start_landmark.y * height), height - 1),
            )
            end_point = (
                min(int(end_landmark.x * width), width - 1),
                min(int(end_landmark.y * height), height - 1),
            )
            cv2.line(image, start_point, end_point, RED_COLOR, 3, cv2.LINE_AA)


def _draw_gesture_text(image: NDArray, landmarks: Landmarks):
    """Draw the gesture text on the image

    Args:
        image (NDArray): The image to draw the gesture text
        landmarks (Landmarks): The landmarks to draw the gesture text
    """
    height, width, _ = image.shape

    if landmarks.left_hand.gesture != "":
        cv2.putText(
            image,
            f"Left Gesture: {landmarks.left_hand.gesture}",
            (10, height - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            GREEN_COLOR,
            1,
            cv2.LINE_AA,
        )

    if landmarks.right_hand.gesture != "":
        cv2.putText(
            image,
            f"Right Gesture: {landmarks.right_hand.gesture}",
            (10, height - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            GREEN_COLOR,
            1,
            cv2.LINE_AA,
        )


def annotate_landmarks(
    image: NDArray,
    landmarks: Landmarks,
) -> NDArray:
    """Anotate the image with the landmarks

    Args:
        image (NDArray): The image to annotate
        landmarks (Landmarks): The landmarks to annotate

    Returns:
        NDArray: The annotated image
    """
    annotated_image = np.copy(image)

    _draw_connections(annotated_image, landmarks)
    _draw_landmark_points(annotated_image, landmarks)
    # _draw_hand_bounding_box(annotated_image, landmarks)
    _draw_gesture_text(annotated_image, landmarks)

    return annotated_image
