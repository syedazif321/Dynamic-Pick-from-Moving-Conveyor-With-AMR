cmake_minimum_required(VERSION 3.8)
project(box_spawner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Find required dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)

# =========================
# Include directories
# =========================
include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# =========================
# Build the plugin
# =========================
add_library(box_spawner_plugin SHARED src/box_spawner_plugin.cpp)
ament_target_dependencies(box_spawner_plugin rclcpp std_srvs gazebo_ros)
target_link_libraries(box_spawner_plugin ${GAZEBO_LIBRARIES})

# =========================
# Install the plugin
# =========================
install(
  TARGETS box_spawner_plugin
  LIBRARY DESTINATION lib
)

# =========================
# Export plugin info
# =========================
ament_export_libraries(box_spawner_plugin)
ament_export_dependencies(rclcpp std_srvs gazebo_ros)

ament_package()
