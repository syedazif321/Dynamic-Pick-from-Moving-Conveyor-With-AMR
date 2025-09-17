cmake_minimum_required(VERSION 3.8)
project(alphabot_navigation)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)   
find_package(nav2_msgs REQUIRED) 


add_executable(map_switcher src/map_switcher.cpp)
ament_target_dependencies(map_switcher rclcpp std_srvs std_msgs nav2_msgs) 


install(TARGETS
  map_switcher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scan_filter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config urdf meshes worlds maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
