cmake_minimum_required(VERSION 3.5)
project(motor_interface)

cmake_policy(SET CMP0148 OLD)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 需要的 ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成服务定义
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VelAndPose.msg"
  "msg/DebugData.msg"
  "srv/GetJointState.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# 包含路径
include_directories(include)

# 安装目标
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
