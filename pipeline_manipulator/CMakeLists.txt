cmake_minimum_required(VERSION 3.8)
project(pipeline_manipulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Find dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# --- For JSON and Nav2 Actions ---
find_package(nav2_msgs REQUIRED) 
find_package(control_msgs REQUIRED)
find_package(msg_gazebo REQUIRED)  # You confirmed this exists
include_directories(/usr/include)

# ------------------------------------------------------------------------------
# Vision Node
# ------------------------------------------------------------------------------
add_executable(vision_node src/vision_node.cpp)
ament_target_dependencies(vision_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  cv_bridge
  OpenCV
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)

target_include_directories(vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------------------------
# Pick Controller Node
# ------------------------------------------------------------------------------
add_executable(pick_controller src/pick_controller.cpp)
ament_target_dependencies(pick_controller
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  moveit_ros_planning_interface
)

target_include_directories(pick_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------------------------
# Pipeline FSM Node (Core)
# ------------------------------------------------------------------------------
add_executable(pipeline_fsm
  src/main.cpp
  src/pipeline_fsm.cpp
)

target_link_libraries(pipeline_fsm
  sqlite3  
)

ament_target_dependencies(pipeline_fsm
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav2_msgs
  control_msgs
  std_srvs
  msg_gazebo

)

target_include_directories(pipeline_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(TARGETS
  vision_node
  pick_controller
  pipeline_fsm
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()