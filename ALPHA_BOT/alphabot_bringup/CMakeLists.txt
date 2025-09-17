cmake_minimum_required(VERSION 3.8)
project(alphabot_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config urdf meshes worlds maps scripts
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)
ament_package()
