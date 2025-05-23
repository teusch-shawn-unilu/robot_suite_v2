# Installation

## Prerequisites

- ROS2 Humble or later (only tested with Humble)
- Ubuntu 22.04

## Automatic Installation (Recommended)

A script is provided to automatically install all dependencies and setup the workspace.
The robot type needs to be specified as an argument to the script. The available options are:

- `tello`: for the Tello drone
- `spot`: for the Spot robot (Not yet supported)
- `unitree_go1`: for the Unitree Go1 robot (Not yet supported)

```bash
sudo ./bootstrap.sh [robot]
```

!!! warning "Running the script"

    Make sure to run the script with `sudo`.

## Manual Installation

If you prefer to install the dependencies manually, you will first need to ensure
your robot driver is installed properly. Additionally, pip packages are required
which can be found in the `requirements.txt` file. You can install them using the following command:

```bash
pip install -r requirements.txt
```

For ros dependencies, you can use the following command:

```bash
rosdep install --from-paths . -r -y
```
