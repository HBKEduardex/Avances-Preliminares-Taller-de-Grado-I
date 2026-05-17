FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    git \
    bash \
    nano \
    tree \
    python3-colcon-common-extensions \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-tf2-tools \
    && rm -rf /var/lib/apt/lists/*
