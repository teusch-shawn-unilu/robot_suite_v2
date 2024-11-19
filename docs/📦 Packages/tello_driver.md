# Tello Driver

The `tello_driver` package is the only one that interacts directly with the drone. It is built on top of the Python library `tellopy`, which implements the low-level protocol for controlling the Tello drone. This protocol was reverse-engineered by the [tellopilots community](https://tellopilots.com/wiki/protocol).

With this package, you can send commands to the drone, such as takeoff, landing, and more. For a full list of available commands, refer to the [topics section](#ros-related).

## Dependencies

- **`tellopy`**: This library must be installed from source to ensure you are using the latest version.
- **`tello_msgs`**: A required package for message definitions.

## 🤖 ROS Related <a id="ros-related"></a>

### 📥 Subscribed Topics

| Topic Name          | Message Type                 | Description                                      |
| ------------------- | ---------------------------- | ------------------------------------------------ |
| `/camera/exposure`  | `std_msgs/msg/Int32`         | Sets camera exposure. _(Valid values: 0, 1, 2)_  |
| `/cmd_vel`          | `geometry_msgs/msg/Twist`    | Command velocity for drone movement.             |
| `/flip`             | `tello_msgs/msg/FlipControl` | Controls flips of the drone.                     |
| `/land`             | `std_msgs/msg/Empty`         | Triggers drone landing.                          |
| `/takeoff`          | `std_msgs/msg/Empty`         | Triggers drone takeoff.                          |
| `/palm_land`        | `std_msgs/msg/Empty`         | Activates palm landing feature.                  |
| `/set_att_limit`    | `std_msgs/msg/Int32`         | Sets altitude limit for the drone. _(in meters)_ |
| `/throw_and_go`     | `std_msgs/msg/Empty`         | Activates throw and go feature.                  |
| `/toggle_fast_mode` | `std_msgs/msg/Empty`         | Toggles the drone's fast mode.                   |

### 📤 Published Topics

| Topic Name          | Message Type                 | Description                                                       |
| ------------------- | ---------------------------- | ----------------------------------------------------------------- |
| `/camera/image_raw` | `sensor_msgs/msg/Image`      | Camera images from the drone.                                     |
| `/flight_data`      | `tello_msgs/msg/FlightStats` | Flight data and statistics.                                       |
| `/imu`              | `sensor_msgs/msg/Imu`        | IMU data of the drone.                                            |
| `/odom`             | `nav_msgs/msg/Odometry`      | Odometry information. _(It is very inacurate. Should be avoided)_ |
| `/battery_state`    | `sensor_msgs/BatteryState`   | Battery State information. (Percentage is from 0-100%)            |

## 🔧 ROS Parameters

Here are the configurable parameters for the `tello_ros_wrapper`:

### Topics

| Parameter Name                | Default Value      | Description                             |
| ----------------------------- | ------------------ | --------------------------------------- |
| `image_topic_name`            | 'camera/image_raw' | Topic name for camera images.           |
| `flight_data_topic_name`      | 'flight_data'      | Topic name for flight data.             |
| `velocity_command_topic_name` | 'cmd_vel'          | Topic name for velocity commands.       |
| `land_topic_name`             | 'land'             | Topic name for landing command.         |
| `takeoff_topic_name`          | 'takeoff'          | Topic name for takeoff command.         |
| `flip_control_topic_name`     | 'flip'             | Topic name for flip control.            |
| `odom_topic_name`             | 'odom'             | Topic name for odometry data.           |
| `imu_topic_name`              | 'imu'              | Topic name for IMU data.                |
| `toggle_fast_mode_topic_name` | 'toggle_fast_mode' | Topic name for toggling fast mode.      |
| `camera_exposure_topic_name`  | 'camera/exposure'  | Topic name for camera exposure control. |

### Frame IDs

| Parameter Name   | Default Value | Description                 |
| ---------------- | ------------- | --------------------------- |
| `imu_frame_id`   | 'imu'         | Frame ID for IMU data.      |
| `odom_frame_id`  | 'odom'        | Frame ID for odometry data. |
| `drone_frame_id` | 'tello'       | Frame ID for the drone.     |

### Wifi Setup

| Parameter Name         | Default Value | Description                          |
| ---------------------- | ------------- | ------------------------------------ |
| `auto_wifi_connection` | false         | Automatically connect to Tello WiFi. |
| `tello_ssid`           |               | SSID for Tello WiFi connection.      |
| `tello_pw`             |               | Password for Tello WiFi.             |

### Settings

| Parameter Name    | Default Value | Description                               |
| ----------------- | ------------- | ----------------------------------------- |
| `alt_limit`       | 30            | Altitude limit in meters.                 |
| `fast_mode`       | false         | Enables fast mode for the drone.          |
| `video_mode`      | '4:3'         | Sets video mode (options: '4:3', '16:9'). |
| `camera_exposure` | 0             | Camera exposure level (0, 1, 2).          |
