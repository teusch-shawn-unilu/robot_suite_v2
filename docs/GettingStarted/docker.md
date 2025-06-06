# Docker

!!! warning

    This is still not up to date!

A `Dockerfile` is provided for using this suite within Docker. This makes it easy to test and use without the need of installing any dependency on your computer nor even install ROS2.
Additionally, a `docker-compose.yml` is also available to streamline the execution of the bringup and any other component that will come in the future.

## Usage

!!! Danger "Important"

    To make sure that you have GUI support, run the following command once `xhost +local:docker`.

### With Docker Compose (Recommended)

!!! info

    Note that you will need to have `docker-compose` installed on your system.
    You can check if it is installed by running `docker-compose -v`. On ubuntu,
    you can install it using `sudo apt install docker-compose`

For an easy to use experience:

1. Go to the root of the project

2. Run `docker-compose`

```bash
docker compose up
```

### Manual

1. Go to the root of the project

1. Build the docker image

```sh
docker build -t tello_suite .
```

3. Create container and run container with bringup (Recommended)

```sh
docker run --rm -it -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY --net=host tello_suite \
    ros2 launch tello_bringup system_launch.py
```

4. Create container and run container with bash

```sh
docker run --rm -it -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY --net=host tello_suite \
    bash
```
