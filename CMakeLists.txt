cmake_minimum_required(VERSION 3.16)
project(kalman_arm_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  rclcpp
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
  rclcpp
)
set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_components
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    trajectory_msgs)

set(TWIST_REPUB_DEPS
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_components
    realtime_tools
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    builtin_interfaces)

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
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${TWIST_REPUB_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()


# Component Nodes (Shared libraries) ############################
set(SERVO_CONTROLLER_INPUT servo_controller_input)


# Add executable for using a controller
add_library(${SERVO_CONTROLLER_INPUT} SHARED reference_generator/joystick_servo_example.cpp)
ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "moveit_servo::JoyToServoPub")


set(MASTER_CAN master_can_node)

add_library(${MASTER_CAN} SHARED hardware/master_hardware.cpp)
ament_target_dependencies(${MASTER_CAN} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${MASTER_CAN} "kalman_master::CanMasterNode")


## COMPILE
add_executable(send_trajectory reference_generator/send_trajectory.cpp)

add_executable(servo_keyboard_input reference_generator/servo_keyboard_input.cpp)
target_include_directories(servo_keyboard_input PUBLIC include)
ament_target_dependencies(servo_keyboard_input ${THIS_PACKAGE_INCLUDE_DEPENDS})


add_executable(twist_republisher reference_generator/twist_republisher.cpp)
target_include_directories(twist_republisher PUBLIC include)
ament_target_dependencies(twist_republisher ${TWIST_REPUB_DEPS})

# add_executable(can_read_test
#   test/can_read_test.cpp
#   hardware/can_driver.cpp
#   hardware/can_handlers.cpp
#   hardware/can_vars.cpp
#   hardware/arm_config.cpp
# )

# add_executable(can_write_test
#   test/can_write_test.cpp
#   hardware/can_driver.cpp
#   hardware/can_handlers.cpp
#   hardware/can_vars.cpp
#   hardware/arm_config.cpp
# )
include_directories(hardware/include)
# ament_target_dependencies(
#   can_read_test PUBLIC
#   ${REF_GEN_INCLUDE_DEPENDS}
# )

# ament_target_dependencies(
#   can_write_test PUBLIC
#   ${REF_GEN_INCLUDE_DEPENDS}
# )


ament_target_dependencies(
  send_trajectory PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

add_library(
  kalman_arm_controller
  SHARED
  hardware/arm_hardware.cpp
  hardware/can_src/arm_config.cpp
  hardware/can_src/can_driver.cpp
  hardware/can_src/arm_handlers.cpp
  hardware/can_src/master_handlers.cpp
  hardware/can_src/can_vars.cpp
  controller/arm_controller.cpp
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
  DIRECTORY meshes
  DESTINATION share/kalman_arm_controller
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/kalman_arm_controller
)
install(
    TARGETS send_trajectory
    RUNTIME DESTINATION lib/kalman_arm_controller
)

# install(TARGETS can_read_test
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS can_write_test
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS servo_keyboard_input
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS twist_republisher
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS kalman_arm_controller
    ${SERVO_CONTROLLER_INPUT}
  EXPORT export_kalman_arm_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

#   ament_add_gtest(${PROJECT_NAME}_read_test test/can_read_test.cpp)
#   target_include_directories(${PROJECT_NAME}_read_test PUBLIC
#     $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
#     $<INSTALL_INTERFACE:include/kalman_arm_controller>
#   )

#   ament_target_dependencies(${PROJECT_NAME}_read_test
#     std_msgs
#   )
#   target_link_libraries(${PROJECT_NAME}_read_test kalman_arm_controller)

  ament_add_pytest_test(example_7_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_7_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_kalman_arm_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS} ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
