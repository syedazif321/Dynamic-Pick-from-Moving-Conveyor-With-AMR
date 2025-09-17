cmake_minimum_required(VERSION 3.5)
project(elevator_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(ElevatorPlugin SHARED src/ElevatorPlugin.cc)
ament_target_dependencies(ElevatorPlugin rclcpp std_srvs)
target_link_libraries(ElevatorPlugin ${GAZEBO_LIBRARIES})


install(TARGETS ElevatorPlugin
  LIBRARY DESTINATION lib
)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugin_path.dsv.in")

ament_package()
