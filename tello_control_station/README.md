# Tello Control Station

## Introduction

Welcome to the Control Station repo !

This module is part of the [tello_ws project](https://github.com/snt-arg/tello_ws).

Its main role is to handle the interactions between the user and the tello drone in order to control it in many possible ways. It is meant to be used with the other plugins present in the tello_ws. 
You can run this node with the command:

```bash
ros2 run tello_control_station control_station
```

Don't forget to build the project and source the `setup.bash` file first !


### Keyboard control mode

The keyboard control mode is always active, allowing you to change the control mode or stop the drone with your keyboard at any time.

When the control mode is set to keyboard, you can also move the drone using your keyboard.

### Joystick control mode

Once a joystick is connected to your computer and the control mode is set to joystick, you can control the tello with a joystick. 

### Hand control mode

Once the control mode is set to hand, you will be able to control the tello with hand signs thanks to our [hand tracker project](https://github.com/snt-arg/hand_tracker_plugin) (follow the instructions of this repo to use this feature).

### Face control mode

Once the control mode is set to face, you will be able to control the tello with your face thanks to our [face tracker project](https://github.com/snt-arg/face_tracker_plugin) (follow the instructions of this repo to use this feature)


## ROS2 Topics 

### Published topics


| Name | Type | Description |
|------|------|-------------|
| `/key_pressed` | String | Publishes the key pressed if this key belongs to the accepted keys and is not related to a movement action|
| `/cmd_vel` | Twist | Drone velocity in the form linear velocity x y z and angular velocity x y z|
| `/takeoff` | Empty | Drone take-off |
| `/land` | Empty | Drone landing |
| `/user_chosen_face` | FaceCoordinates| Coordinates of the face clicked (only when the control mode is set to face)|

### Subscribed topics

| Name | Type | Description |
|------|------|-------------|
| `/flight_data` | FlightData | Used to get the battery level of the drone|
| `/camera/image_raw` | Image | Images from the camera of the drone|
| `/faces_list` | FaceList | List of the face detected (only when the face detection is running)|
| `/face_to_follow` | FaceCoordinates | Coordinates fo the face to follow (only when the face estimator is running)|

## Control the drone

### Keyboard Keys

| Key | Action |
|-----|--------|
| `j` | Switch to joystick control mode|
| `k` | Switch to keyboard control mode|
| `h` | Swtich to hand control mode|
| `f` | Switch to face control mode|
| `m` | Switch to manual control mode|
| `t` | Take-off |
| `l` | Land |
| `q` | Quit |
| `e` | Emergency |
| `w` | Go forward |
| `s` | Go backward |
| `a` | Go left |
| `d` | Go right |
| `up arrow` | Go up |
| `down arrow` | Go down |
| `left arrow` | Turn left |
| `right arrow` | Turn right |

### Joystick buttons

| Button | Direction | Action |
|--------|-----------|--------| 
| `Right analog` | Right | Go right |
| | Left | Go left |
| | Up | Go forward |
| | Down | Go backward|
| `Left analog` | Right | Turn right |
| | Left | Turn left |
| | Up | Go up |
| | Down | Go down |
| `A` | | Switch to hand control mode |
| `B` | | Switch to face control mode |
| `X` | | Switch to keyboard control mode |
| `LB` | | Emergency |
| `RB` | | Quit |
| `Back` | | Land |
| `Start` | | Take-off |

note: these commands are based on the Logitech F710 Gamepad, the inputs may change if you use an other one.


