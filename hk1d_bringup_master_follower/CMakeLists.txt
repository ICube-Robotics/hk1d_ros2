cmake_minimum_required(VERSION 3.5)
project(hk1d_bringup_master_follower)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config ros2_control launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
