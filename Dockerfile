FROM ros:humble

# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 

# Install necessary and useful packages
RUN apt update && \
    apt install --no-install-recommends -y \
    ssh vim python3-pip python3-vcstool wget git \
    ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-rmw-cyclonedds-cpp 

# Install Tellopy
WORKDIR /tmp
RUN git clone https://github.com/hanyazou/TelloPy.git tellopy
WORKDIR /tmp/tellopy
RUN pip install .
WORKDIR /tmp
RUN rm -rf tellopy

# Install PIP libaries
RUN pip install av pillow py-trees

# Copy Tello Suite
WORKDIR /workspace/src
COPY ./tello_suite/ ./tello_suite
COPY ./tello_driver/ ./tello_driver
COPY ./tello_msgs/ ./tello_msgs
COPY ./tello_plugins/ ./tello_plugins

# Colcon build + source
WORKDIR /workspace
RUN . /opt/ros/humble/setup.sh && colcon build

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc
