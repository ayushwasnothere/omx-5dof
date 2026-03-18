cmake_minimum_required(VERSION 3.8)
project(open_manipulator_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(omx_interfaces REQUIRED)

add_executable(manipulator_node src/manipulator_node.cpp)
ament_target_dependencies(manipulator_node
  rclcpp
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
  omx_interfaces
)
target_include_directories(manipulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS manipulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(PROGRAMS scripts/camera_tf_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()