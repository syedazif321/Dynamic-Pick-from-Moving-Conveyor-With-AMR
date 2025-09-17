cmake_minimum_required(VERSION 3.5)
project(object_spawn_plugin)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

# Include Gazebo headers
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Build the plugin
add_library(${PROJECT_NAME} SHARED src/random_spawner_plugin.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  gazebo_ros
)

target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
)

# Install the plugin so Gazebo can find it
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

ament_package()
