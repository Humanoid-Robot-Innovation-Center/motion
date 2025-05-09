cmake_minimum_required(VERSION 3.8)
project(gongga_interface)

cmake_policy(SET CMP0148 OLD)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GeneratePlan.srv"
  "srv/ExecuteTrajectory.srv"
  "srv/CommandService.srv"
  "srv/SceneSync.srv"
  "action/FollowJointTrajectory.action"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
