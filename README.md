# Tello Suite

> [!IMPORTANT]
> This is still a work in progress. Expect bugs!
> If you find any, please create an issue or submit a PR fixing it.

Tello suite is a collection of ros2 packages which makes the tello more intelligent.
It's main target is to make use of plugins (which are ros2 packages) in order to add
or delete functionality.

In it's core lies the `tello_bt`, which is a behavior tree which decdies what runs and when.

For now, this suites offers a fully featured tello driver which

## Components

- `tello_driver`: A fully featured tello driver which can be used to control the drone
  as well as received image data or drone status, such as battery. For more information, check
  the [package](./tello_driver/).
- `tello_msgs`: A set of custom messages/services package for the tello.
- `tello_bringup`: A bring-up package which facilitates the launch of the various packages all at once.
- `plugin_server_base`: This class serves as a basis for your plugin package. This means that you can use `PluginBase` instead of
  `rclpy.Node`. Check the package [README](./plugin_server_base/README.md)
- `tello_bt`: The behavior tree for the tello suite, which commands when a plugin should run
  and more.
- `tello_control_station`: An application which can be used to control the tello using keyboard
  and joystick. In addition, it serves as a interface to visualize or interact with a plugin for example. (**NOTE: This package will be soon deprecated in favor of `tello_app`**)
- `tello_plugins`: Contains the set of plugins packages that can be used.
- `tello_nav`: Not yet implemented -> Give the tello a way to localize itself while building a map, using VSLAM technology
- `tello_app`: Partially implemented -> A web-based tello application that will be replacing the `tello_control_station`

## Dependencies

- tellopy (needs to be installed from source)
- av
- pillow
- py-trees
- pygame
- mediapipe
