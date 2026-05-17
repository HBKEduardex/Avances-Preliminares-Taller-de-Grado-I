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
    ros-humble-moveit \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-plugins \
    && rm -rf /var/lib/apt/lists/*
