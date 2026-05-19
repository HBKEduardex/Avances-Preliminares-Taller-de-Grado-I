cmake_minimum_required(VERSION 3.8)
project(kuka_pick_place_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SavePoint.srv"
)

ament_package()
