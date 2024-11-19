# Hand Gestures Plugin

This plugin aims to detect hand landmarks and interpret the landmarks to gestures.
These gestures can then be used to control a robot movement for instance.

In addition, the plugin is meant to be used in conjunction with a behaviour tree like the [tello_bt](https://github.com/snt-arg/tello_bt).
Nevertheless, it can be used in a standalone mode.

## 🧩 Dependencies

- [`tello_plugin_utils`](https://github.com/snt-arg/tello_plugin_utils).

## 🚀 Standalone Usage

!!! note

    Each command needs to be executed in 2 different terminals and you must source the `install/setup.bash` which was created in the section above.

1. Launch the detector node in **standalone** mode

```sh
 ros2 run hand_gestures landmark_detector_node --ros-args -p standalone:=true
```

2. Launch the annotator node in **standalone** mode

```sh
 ros2 run hand_gestures landmark_annotator_node --ros-args -p standalone:=true
```

## 🏗️ Architecture

![hand gestures architecture](../assets/hand_gestures_architecture.png){align=center}

The main idea is to split work into multiple nodes. We first have a `detector` node, which only goal is to detect the hand landmarks and publish them to a topic `/hand/landmarks`.

In addition, we have another node `annotator` which receives the same images which are fed into the detector node and also the landmarks produced by the detector node. Then, it will draw the detected landmarks into the image and publish the annotated image to the topic `/hand/annotated/image`.

To conclude the hand gesture package, we have the `sign_classifier` node. It's goal is to receive the hand landmarks and interpret those into hand gestures.

In order to support multiple robots, it is needed to create an extra node which will be depended to the robot itself. This node takes in the interpreted gestures and then translates them into velocity commands and others, like stand up, take-off, etc.

## 🤖 ROS Related

### 📥 Subscribed Topics

#### 🌐 Landmark Detector Node

| Topic Name          | Message Type                                                                             | Description                     |
| ------------------- | ---------------------------------------------------------------------------------------- | ------------------------------- |
| `/camera/image_raw` | [sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)) | Input image for hand detection. |

#### 🌐 Landmark Annotator Node

| Topic Name          | Message Type                                                                                                                    | Description                         |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------- |
| `/camera/image_raw` | [sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))                                        | Input image use for hand detection. |
| `/hand/landmarks`   | [hand_gestures_msgs/Landmakrs](https://github.com/snt-arg/hand_gestures_plugin/blob/main/hand_gestures_msgs/msg/Landmarks.msg)) | Landmarks detected.                 |

### 📤 Published Topics

#### 🌐 Landmark Detector Node

| Topic Name        | Message Type                                                                                                                    | Description                                |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------ |
| `/hand/landmarks` | [hand_gestures_msgs/Landmakrs](https://github.com/snt-arg/hand_gestures_plugin/blob/main/hand_gestures_msgs/msg/Landmarks.msg)) | Output landmarks topic for detected hands. |

#### 🌐 Landmark Annotator Node

| Topic Name              | Message Type                                                                             | Description                                        |
| ----------------------- | ---------------------------------------------------------------------------------------- | -------------------------------------------------- |
| `/hand/annotated/image` | [sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)) | Annotated image with landmarks for detected hands. |

### ⚙️ Parameters

#### 🌐 Landmark Detector Node

| Parameter Name             | Description                          | Default             |
| -------------------------- | ------------------------------------ | ------------------- |
| `img_input_topic`          | Input image topic name.              | `/camera/image_raw` |
| `landmarks_topic`          | Detected landmarks on image.         | `hand/landmarks`    |
| `num_hands`                | Maximum number of hands to detect.   | 2                   |
| `min_detection_confidence` | Minimum confidence to detect a hand. | 0.5                 |
| `min_tracking_confidence`  | Minimum confidence to track a hand.  | 0.5                 |

#### 🌐 Landmark Annotator Node

| Parameter Name        | Description                                | Default             |
| --------------------- | ------------------------------------------ | ------------------- |
| `img_input_topic`     | Input image topic name.                    | `/camera/image_raw` |
| `annotated_img_topic` | Annotated image with landmarks topic name. | `/camera/image_raw` |
| `landmarks_topic`     | Detected landmarks on image.               | `hand/landmarks`    |
