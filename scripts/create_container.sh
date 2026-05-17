#!/bin/bash
docker create -it \
  --name kuka_ros2_humble_container \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$(pwd)":/root/taller1 \
  kuka_ros2_humble:dev
