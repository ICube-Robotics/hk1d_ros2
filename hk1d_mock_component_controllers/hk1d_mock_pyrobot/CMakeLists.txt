cmake_minimum_required(VERSION 3.16)
project(hk1d_mock_pyrobot LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Set the parameter header file name
set(PARAM_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/hk1d_mock_pyrobot_parameters/include)
set(PARAM_HEADER_FILE ${PARAM_INCLUDE_DIR}/hk1d_mock_pyrobot_parameters.hpp)

# Make logic build directory
set(LOGIC_DIR ${CMAKE_CURRENT_BINARY_DIR}/${LIB_NAME}/hk1d_mock_pyrobot_logic)
set(LOGIC_INCLUDE_DIR ${LOGIC_DIR}/include/hk1d_mock_pyrobot)
file(MAKE_DIRECTORY ${LOGIC_DIR})
file(MAKE_DIRECTORY ${LOGIC_INCLUDE_DIR})

file (REMOVE ${LOGIC_DIR}/hk1d_mock_pyrobot_logic.cpp)
file (REMOVE ${LOGIC_DIR}/hk1d_mock_pyrobot_logic.h)
file (REMOVE ${LOGIC_INCLUDE_DIR}/hk1d_mock_pyrobot_logic.h)

add_custom_command(
  OUTPUT ${LOGIC_DIR}/hk1d_mock_pyrobot_logic.cpp ${LOGIC_DIR}/hk1d_mock_pyrobot_logic.h
  COMMAND cython3 -3 --cplus ${CMAKE_CURRENT_SOURCE_DIR}/src/hk1d_mock_pyrobot_logic.pyx
    -o ${LOGIC_DIR}/hk1d_mock_pyrobot_logic.cpp -I ${PARAM_INCLUDE_DIR}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hk1d_mock_pyrobot_logic.pyx
    ${CMAKE_CURRENT_SOURCE_DIR}/script/hk1d_mock_pyrobot_logic_impl.py
)

# Copy the header file into the include directory
add_custom_command(
  OUTPUT ${LOGIC_INCLUDE_DIR}/hk1d_mock_pyrobot_logic.h
  COMMAND ${CMAKE_COMMAND} -E copy
    ${LOGIC_DIR}/hk1d_mock_pyrobot_logic.h ${LOGIC_INCLUDE_DIR}/hk1d_mock_pyrobot_logic.h
  DEPENDS ${LOGIC_DIR}/hk1d_mock_pyrobot_logic.h
)

generate_parameter_library(
  hk1d_mock_pyrobot_parameters
  src/hk1d_mock_pyrobot_parameters.yaml
)

file (REMOVE ${PARAM_INCLUDE_DIR}/hk1d_mock_pyrobot_parameters.pxd)

# Generate the pxd for the library
add_custom_command(
  OUTPUT ${PARAM_INCLUDE_DIR}/hk1d_mock_pyrobot_parameters.pxd
  COMMAND ros2 run pytroller_tools generate_pxd ${PARAM_INCLUDE_DIR}/hk1d_mock_pyrobot_parameters.pxd ${PARAM_HEADER_FILE}
  DEPENDS ${PARAM_HEADER_FILE}
)

add_library(hk1d_mock_pyrobot SHARED
  src/hk1d_mock_pyrobot.cpp
  ${LOGIC_DIR}/hk1d_mock_pyrobot_logic.cpp
  ${LOGIC_INCLUDE_DIR}/hk1d_mock_pyrobot_logic.h
  ${PARAM_INCLUDE_DIR}/hk1d_mock_pyrobot_parameters.pxd
)
target_compile_features(hk1d_mock_pyrobot PUBLIC cxx_std_17)
target_include_directories(hk1d_mock_pyrobot PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/hk1d_mock_pyrobot>
)
target_include_directories(hk1d_mock_pyrobot PUBLIC
  $<BUILD_INTERFACE:${LOGIC_DIR}/include>
  $<INSTALL_INTERFACE:include/hk1d_mock_pyrobot>
)
target_link_libraries(hk1d_mock_pyrobot PUBLIC
  hk1d_mock_pyrobot_parameters
  ${PYTHON_LIBRARIES}
)
ament_target_dependencies(hk1d_mock_pyrobot PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hk1d_mock_pyrobot PRIVATE "PYTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Load test
  add_rostest_with_parameters_gmock(
    test_load_hk1d_mock_pyrobot
    test/test_load_hk1d_mock_pyrobot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml
  )
  target_link_libraries(test_load_hk1d_mock_pyrobot
    hk1d_mock_pyrobot
  )
  ament_target_dependencies(test_load_hk1d_mock_pyrobot
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  # Controller test
  add_rostest_with_parameters_gmock(
    test_hk1d_mock_pyrobot
    test/test_hk1d_mock_pyrobot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml
  )

  target_link_libraries(test_hk1d_mock_pyrobot
    hk1d_mock_pyrobot
  )

  ament_target_dependencies(test_load_hk1d_mock_pyrobot
    controller_manager
    hardware_interface
  )

endif()

install(
  DIRECTORY ${LOGIC_DIR}/include
  DESTINATION include/hk1d_mock_pyrobot
)
install(
  DIRECTORY include/
  DESTINATION include/hk1d_mock_pyrobot
)
install(
  TARGETS
    hk1d_mock_pyrobot
    hk1d_mock_pyrobot_parameters
  EXPORT export_hk1d_mock_pyrobot
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_hk1d_mock_pyrobot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
