cmake_minimum_required(VERSION 3.5)
project(motor_controller)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(motor_interface REQUIRED)

include_directories(
  include
  ${tf2_INCLUDE_DIRS}
)


# 找到modbus库
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(MODBUS_LIB_PATH "/usr/lib/x86_64-linux-gnu")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(MODBUS_LIB_PATH "/usr/lib/aarch64-linux-gnu")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
find_library(MODBUS_LIB NAMES modbus PATHS ${MODBUS_LIB_PATH})


# Declare the library
add_library(lichuan_motor_controller
  src/lichuan_stepper_motor/motor_controller.cpp
)
ament_target_dependencies(lichuan_motor_controller
  rclcpp
  # librcl
)
target_link_libraries(lichuan_motor_controller ${MODBUS_LIB})
target_include_directories(lichuan_motor_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


add_executable(lichuan_driver_node
  src/lichuan_stepper_motor/main.cpp
  src/lichuan_stepper_motor/driver_node.cpp
)
ament_target_dependencies(lichuan_driver_node
  rclcpp
  std_msgs
  geometry_msgs
  motor_interface
  tf2
  tf2_ros
  nav_msgs
)
target_link_libraries(lichuan_driver_node
  lichuan_motor_controller
  ${MODBUS_LIB}
)


add_library(SMS_STS
    src/feetech_servo_motor/SMS_STS.cpp
    src/feetech_servo_motor/SCSerial.cpp
    src/feetech_servo_motor/SCS.cpp
)
ament_target_dependencies(SMS_STS
    rclcpp
)
target_include_directories(SMS_STS PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


# Declare the executable
add_executable(feetech_driver_node
  src/feetech_servo_motor/motor_controller_node.cpp
)
ament_target_dependencies(feetech_driver_node
  rclcpp
  std_msgs
  yaml-cpp
  motor_interface
  tf2
  tf2_ros
)
target_link_libraries(feetech_driver_node
  yaml-cpp
  SMS_STS
)


add_executable(joint_state_publisher_node
  src/joint_state_publisher.cpp
)
ament_target_dependencies(joint_state_publisher_node
  rclcpp
  std_msgs
  motor_interface
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(joint_state_publisher_node
  SMS_STS
  lichuan_motor_controller
)


# Install targets
install(TARGETS
    lichuan_motor_controller
    SMS_STS
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
    lichuan_driver_node
    feetech_driver_node
    joint_state_publisher_node
    DESTINATION lib/${PROJECT_NAME}
)


# Install header files
install(
  DIRECTORY include/
  DESTINATION include/
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export targets and libraries
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(motor_controller SMS_STS)
ament_export_dependencies(rclcpp motor_interface)
ament_package()
