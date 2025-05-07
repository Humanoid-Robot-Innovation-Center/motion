cmake_minimum_required(VERSION 3.5)
project(graspnet_generator)

# 查找 Python 依赖
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 查找和生成服务文件
find_package(rosidl_default_generators REQUIRED)

# 声明服务接口（.srv）
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPose.srv"  # 你的服务文件路径
  DEPENDENCIES std_msgs geometry_msgs  # 依赖的消息包
)

# 安装 Python 文件
install(PROGRAMS
  src/graspnet_generator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 配置包
ament_package()
