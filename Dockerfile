FROM ros:humble

# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 

# Install necessary and useful packages
RUN apt update && \
    apt install --no-install-recommends -y \
    ssh vim python3-pip python3-vcstool wget git iputils-ping\
    ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-rmw-cyclonedds-cpp 

# Copy Tello Suite
COPY . /workspace/src/tello_suite

# Install Tellopy
WORKDIR /tmp
RUN git clone https://github.com/hanyazou/TelloPy.git tellopy
WORKDIR /tmp/tellopy
RUN pip install .
WORKDIR /tmp
RUN rm -rf tellopy

# Install PIP libaries
WORKDIR /workspace/src/tello_suite
RUN pip install -r requirements.txt

# Colcon build + source
WORKDIR /workspace
RUN . /opt/ros/humble/setup.sh && colcon build

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

WORKDIR /workspace/src/tello_suite
RUN chmod 755 entryfile.sh
ENTRYPOINT ["bash", "entryfile.sh"]
