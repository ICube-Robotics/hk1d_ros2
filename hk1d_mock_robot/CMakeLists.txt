cmake_minimum_required(VERSION 3.16)
project(hk1d_mock_robot LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  angles
  control_msgs
  control_toolbox
  controller_interface
  Eigen3
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(hk1d_mock_robot_parameters
  src/hk1d_mock_robot_parameters.yaml
)

add_library(hk1d_mock_robot SHARED
  src/hk1d_mock_robot.cpp
)
target_compile_features(hk1d_mock_robot PUBLIC cxx_std_17)
target_include_directories(hk1d_mock_robot PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hk1d_mock_robot PUBLIC
  hk1d_mock_robot_parameters
)
ament_target_dependencies(hk1d_mock_robot PUBLIC ${DEPENDENCIES})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hk1d_mock_robot PRIVATE "HK1D_MOCK_ROBOT_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface hk1d_mock_robot.xml)


install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS hk1d_mock_robot hk1d_mock_robot_parameters
  EXPORT export_hk1d_mock_robot
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_hk1d_mock_robot HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
