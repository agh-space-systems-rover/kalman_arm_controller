cmake_minimum_required(VERSION 3.16)
project(kalman_arm_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_executable(send_trajectory reference_generator/send_trajectory.cpp)
## UNIT TESTS
add_executable(read_test unittests/can_read_test.cpp)
add_executable(write_test unittests/can_write_test.cpp)

ament_target_dependencies(
  send_trajectory PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

add_library(
  kalman_arm_controller
  SHARED
  hardware/r6bot_hardware.cpp
  hardware/can_driver.cpp
  hardware/can_handlers.cpp
  controller/r6bot_controller.cpp
)

target_compile_features(kalman_arm_controller PUBLIC cxx_std_17)
target_include_directories(kalman_arm_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/kalman_arm_controller>
)
ament_target_dependencies(
  kalman_arm_controller PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KALMAN_ARM_CONTROLLER_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface kalman_arm_controller.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface kalman_arm_controller.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/kalman_arm_controller
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/kalman_arm_controller
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/kalman_arm_controller
)
install(
    TARGETS send_trajectory read_test write_test
    RUNTIME DESTINATION lib/kalman_arm_controller
)

install(TARGETS kalman_arm_controller
  EXPORT export_kalman_arm_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_7_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_7_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_kalman_arm_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
