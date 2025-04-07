# 🚁 Tello Suite

> [!IMPORTANT]
> This project is still a work in progress, so expect some bugs!  
> If you encounter any, please open an issue or submit a PR with a fix.

Tello Suite is a collection of ROS2 packages designed to enhance the intelligence of the DJI Tello drone. The suite leverages plugins (additional ROS2 packages) with distinct functionalities to expand the drone's capabilities.

At its core is `robot_bt`, a behavior tree that orchestrates all components by defining behaviors. This makes the system more robust, allowing you to control which plugins should run and making it more reliable in case of unexpected events.

The suite also includes a `Dockerfile` for easy setup without needing local installation. For more information, see the [Docker](#docker) section.

## Documentation

**Documentation can be found [here](https://snt-arg.github.io/tello_suite/).**

Additionally, the documentation can be viewed locally using the following options:

1.  With docker: `docker run --rm -it -p 8000:8000 -v ${PWD}:/docs squidfunk/mkdocs-material`.
2.  With python: First install material mkdocs with `pip install mkdocs-material`. Then, to preview the documentation run `mkdocs serve`.

> [!NOTE]
> The documentation should become available on [http://localhost:8000](http://localhost:8000)
