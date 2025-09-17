cmake_minimum_required(VERSION 3.8)
project(alphabot_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf   DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY RealSense_D435  DESTINATION share/${PROJECT_NAME})
# alphabot_description/CMakeLists.txt
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_model_path.dsv.in")

ament_package()
