cmake_minimum_required(VERSION 3.8)
project(omx_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_package_xml()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SceneObject.msg"
  "srv/Pick.srv"
  "srv/Place.srv"
  "srv/Pour.srv"
  "srv/Rotate.srv"
  "srv/GoHome.srv"
  "srv/AddObjects.srv"
  "srv/RemoveObjects.srv"
  "srv/GetObjects.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()