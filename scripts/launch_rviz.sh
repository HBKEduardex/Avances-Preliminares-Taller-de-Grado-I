#!/bin/bash
cd /root/taller1/ros2_ws
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi
ros2 launch kuka_kr6_support display.launch.py
