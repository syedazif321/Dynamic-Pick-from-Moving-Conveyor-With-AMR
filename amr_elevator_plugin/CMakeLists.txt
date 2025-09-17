cmake_minimum_required(VERSION 3.8)
project(amr_elevator_plugin)

# Gazebo Classic + ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

add_library(amr_elevator_trigger_plugin SHARED
  src/elevator_trigger_plugin.cpp
)

ament_target_dependencies(amr_elevator_trigger_plugin
  rclcpp
  std_srvs
)

install(TARGETS amr_elevator_trigger_plugin
  DESTINATION lib
)


ament_package()
