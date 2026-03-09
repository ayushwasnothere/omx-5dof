cmake_minimum_required(VERSION 3.8)
project(omx_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Uncomment when adding perception service later:
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/GetPlanningScene.srv"
#   DEPENDENCIES moveit_msgs sensor_msgs
# )

ament_package()