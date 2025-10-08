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
find_package(nav_msgs REQUIRED)
find_package(msg_gazebo REQUIRED)
find_package(nav2_msgs REQUIRED) 
find_package(control_msgs REQUIRED)
find_package(SQLite3 REQUIRED) 
find_package(conveyorbelt_msgs REQUIRED) # Dependency for ConveyorBeltControl service

# ------------------------------------------------------------------------------
# Vision Node (vision_node)
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
  msg_gazebo
)

target_include_directories(vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vision_node ${OpenCV_LIBS})

# ------------------------------------------------------------------------------
# Pick Controller Node (dynamic_pick_controller)
# ------------------------------------------------------------------------------
add_executable(dynamic_pick_controller src/pick_controller.cpp)
ament_target_dependencies(dynamic_pick_controller
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  moveit_ros_planning_interface
  tf2_ros                             
  tf2_geometry_msgs                   
  nav_msgs                            
  msg_gazebo                          
)

target_include_directories(dynamic_pick_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(dynamic_pick_controller
  ${moveit_ros_planning_interface_LIBRARIES} 
  ${msg_gazebo_LIBRARIES}
)

# ------------------------------------------------------------------------------
# Pipeline Setup Node (pipeline_fsm) <-- MODIFIED BLOCK
# This node executes the user-triggered AMR and Conveyor start sequence.
# ------------------------------------------------------------------------------
add_executable(pipeline_fsm src/pipeline_fsm.cpp)

ament_target_dependencies(pipeline_fsm
  rclcpp
  geometry_msgs
  std_srvs
  conveyorbelt_msgs # Dependency for the ConveyorBeltControl service
)

target_include_directories(pipeline_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# You can optionally link other libraries if needed for logging/threading/etc.
# target_link_libraries(pipeline_fsm) 


# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(TARGETS
  vision_node
  dynamic_pick_controller
  pipeline_fsm  # <-- Now installing the new pipeline node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()