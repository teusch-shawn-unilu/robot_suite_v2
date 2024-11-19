# Tello Messages

The `tello_msgs` package defines custom messages and services for communication with the DJI Tello drone. These definitions facilitate control commands and telemetry data exchange, enabling robust interaction with the drone.

---

## Message Types

### `FlightStats.msg`

This message contains detailed telemetry data and status information from the drone, grouped into categories:

#### Battery Data

| Field Name            | Type  | Description                                       |
| --------------------- | ----- | ------------------------------------------------- |
| `battery_low`         | int16 | Indicates if the battery is low.                  |
| `battery_lower`       | int16 | Indicates if the battery is critically low.       |
| `battery_percentage`  | int16 | Current battery level as a percentage.            |
| `drone_battery_left`  | int16 | Remaining battery in the drone.                   |
| `drone_fly_time_left` | int16 | Estimated remaining flight time based on battery. |

#### States

| Field Name                   | Type  | Description                                       |
| ---------------------------- | ----- | ------------------------------------------------- |
| `battery_state`              | int16 | Current battery state.                            |
| `camera_state`               | int16 | Status of the camera.                             |
| `electrical_machinery_state` | int16 | Status of the drone’s motors.                     |
| `down_visual_state`          | int16 | Status of the downward visual positioning system. |
| `gravity_state`              | int16 | Status of gravity sensors.                        |
| `imu_calibration_state`      | int16 | Status of IMU calibration.                        |
| `imu_state`                  | int16 | Status of the IMU.                                |
| `power_state`                | int16 | Status of the power system.                       |
| `pressure_state`             | int16 | Status of pressure sensors.                       |
| `wind_state`                 | int16 | Status of wind-related adjustments.               |

#### Stats

| Field Name     | Type  | Description                                 |
| -------------- | ----- | ------------------------------------------- |
| `drone_hover`  | int16 | Indicates if the drone is hovering.         |
| `em_open`      | int16 | Emergency state (open).                     |
| `em_sky`       | int16 | Emergency state (sky-related).              |
| `em_ground`    | int16 | Emergency state (ground-related).           |
| `factory_mode` | int16 | Indicates if the drone is in factory mode.  |
| `fly_mode`     | int16 | Current flight mode.                        |
| `fly_time`     | int16 | Total flight time in seconds.               |
| `front_in`     | int16 | Status of front sensors (in proximity).     |
| `front_lsc`    | int16 | Front LSC sensor state.                     |
| `front_out`    | int16 | Status of front sensors (out of proximity). |

#### Sensors

| Field Name         | Type  | Description                         |
| ------------------ | ----- | ----------------------------------- |
| `fly_speed`        | int16 | Current flight speed.               |
| `east_speed`       | int16 | Horizontal speed towards the east.  |
| `ground_speed`     | int16 | Speed relative to the ground.       |
| `height`           | int16 | Current height in meters.           |
| `light_strength`   | int16 | Strength of detected light.         |
| `north_speed`      | int16 | Horizontal speed towards the north. |
| `temperature_high` | int16 | Current temperature of the drone.   |

#### Other

| Field Name              | Type  | Description                               |
| ----------------------- | ----- | ----------------------------------------- |
| `outage_recording`      | int16 | Indicates if there are recording outages. |
| `smart_video_exit_mode` | int16 | Status of the smart video exit mode.      |
| `throw_fly_timer`       | int16 | Timer for throw-and-fly mode.             |

#### WiFi

| Field Name      | Type  | Description                   |
| --------------- | ----- | ----------------------------- |
| `wifi_disturb`  | int16 | Level of WiFi disturbance.    |
| `wifi_strength` | int16 | Current WiFi signal strength. |

---

### `FlipControl.msg`

This message is used to control the drone's flips. It defines boolean fields for all possible flip directions:

| Field Name           | Type | Description                             |
| -------------------- | ---- | --------------------------------------- |
| `flip_forward`       | bool | Perform a forward flip.                 |
| `flip_backward`      | bool | Perform a backward flip.                |
| `flip_right`         | bool | Perform a right flip.                   |
| `flip_left`          | bool | Perform a left flip.                    |
| `flip_forward_left`  | bool | Perform a forward-left diagonal flip.   |
| `flip_forward_right` | bool | Perform a forward-right diagonal flip.  |
| `flip_back_left`     | bool | Perform a backward-left diagonal flip.  |
| `flip_back_right`    | bool | Perform a backward-right diagonal flip. |

---

## Services

The `tello_msgs` package also includes services for controlling the drone. These services will be documented in detail as they are implemented.

---

## Usage

This package is essential for ROS2 nodes interacting with the Tello drone. It defines all necessary message and service formats for controlling the drone and processing its telemetry.

For more information, refer to the [Tello Driver documentation](#tello-driver).
