FROM ros:jazzy

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 

# Install necessary and useful packages
RUN apt update && \
    apt install --no-install-recommends -y \
    ssh vim python3-pip python3-vcstool wget git iputils-ping\
    ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-rmw-cyclonedds-cpp python3.12-venv

COPY . /workspace/src/robot_suite

# Install PIP libaries
WORKDIR /workspace/src/robot_suite
RUN pip install -r requirements.txt --break-system-packages

# Run rosdep
WORKDIR /workspace
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Colcon build + source
WORKDIR /workspace
RUN . /opt/ros/jazzy/setup.sh && colcon build

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc


# Build Entrypoint

RUN echo "#!/bin/bash" >> /entrypoint.sh \
    && echo "source /opt/ros/jazzy/setup.bash" >> /entrypoint.sh \
    && echo "source /workspace/install/setup.bash" >> /entrypoint.sh \
    && echo 'exec "$@"' >> /entrypoint.sh \
    && chmod a+x /entrypoint.sh

WORKDIR /workspace/src/robot_suite

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
