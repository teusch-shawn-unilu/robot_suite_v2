# 🚁 Tello Suite

!!! danger

    This project is still a work in progress, so expect some bugs! If you encounter any, please open an issue or submit a PR with a fix.

Tello Suite is a set of ROS2 packages that make the DJI Tello drone smarter and more capable. It uses plugins—extra ROS2 packages with specific features—to extend what the drone can do.

Important packages in the suite include `tello_driver`, which connects to the drone, and `robot_bt`, which organizes how the drone behaves using a behavior tree. This setup makes the system more reliable by letting you control which plugins to use and handling unexpected issues better.

The suite also includes a `Dockerfile` for easy setup without needing to install everything on your computer. For more details, check the [Docker](3.docker.md) page.
