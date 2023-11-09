cmake_minimum_required(VERSION 3.5)
project(hk1d_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY rviz ros2_control urdf srdf gazebo config meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
