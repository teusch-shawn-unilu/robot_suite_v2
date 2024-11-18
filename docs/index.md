# 🚁 Tello Suite

!!! important

    This project is still a work in progress, so expect some bugs! If you encounter any, please open an issue or submit a PR with a fix.

Tello Suite is a collection of ROS2 packages designed to enhance the intelligence of the DJI Tello drone. The suite leverages plugins (additional ROS2 packages) with distinct functionalities to expand the drone's capabilities.

At its core is `tello_bt`, a behavior tree that orchestrates all components by defining behaviors. This makes the system more robust, allowing you to control which plugins should run and making it more reliable in case of unexpected events.

The suite also includes a `Dockerfile` for easy setup without needing local installation. For more information, see the [Docker](#docker) section.

## 🔧 Structure <a id="structure"></a>

- **[tello_driver](./tello_driver)**: A complete Tello driver package for controlling the drone, publishing image data, and tracking status like battery life.
- **[tello_msgs](./tello_msgs)**: Contains custom message and service definitions.
- **[tello_bringup](./tello_bringup)**: A bring-up package that helps launch the various components. Includes `system_launch.py`, which launches everything with the `hand_gestures` plugin.
- **[plugin_server_base](./plugin_server_base)**: A base class for developing plugins. It acts as a bridge between a plugin and the `tello_bt` package.
- **[tello_bt](./tello_bt)**: Implements the behavior tree that orchestrates all components.
- **[tello_control_station](./tello_control_station)**: An interface for controlling the Tello via keyboard and joystick, and for interacting with plugins. (**Note**: This package will soon be deprecated in favor of `tello_app`.)
- **[tello_plugins](./tello_plugins)**: Contains various plugins. Check each plugin package for details on its functionality.
- **[tello_nav](./tello_nav)**: Enables the Tello to localize itself and build maps using VSLAM technology. (**Not yet implemented: potential BSP**)
- **[tello_app](./tello_app)**: A web-based application that will eventually replace `tello_control_station`. (**Partially implemented in another repository**)
