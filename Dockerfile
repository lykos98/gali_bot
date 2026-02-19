FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
    ros-noetic-turtlebot3-slam \
    ros-noetic-turtlebot3-teleop \
    ros-noetic-gmapping \
    ros-noetic-navigation \
    ros-noetic-explore-lite \
    ros-noetic-slam-toolbox \
    ros-noetic-turtlebot3-navigation \
    ros-noetic-geometry-msgs\
    ros-noetic-sensor-msgs\
    ros-noetic-rviz \
    net-tools \
    iputils-ping \
    python3-rosdep \
    python3-pip \
    python3-tk \
    vim 

RUN pip3 install --no-cache-dir \
    numpy \
    matplotlib \
    scipy \
    pandas \
    pyyaml
