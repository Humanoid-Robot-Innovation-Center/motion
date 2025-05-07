# GraspNet Generator

`graspnet_generator` 是一个基于 ROS 2 的抓取任务生成器，用于从图像、深度图像和掩码图像中生成抓取位姿。它结合了 GraspNet 抓取网络和 ROS 2 服务架构，能够接收来自客户端的请求，生成抓取位姿，并通过服务返回结果。

## 功能概述

- 接收 RGB 图像、深度图像和掩码图像来生成抓取位姿。
- 使用 GraspNet 网络模型进行抓取位姿预测。
- 提供服务接口 (`/get_grasp_poses`)，接受请求并返回多个抓取位姿。
- 支持集成运动规划系统和机器人的控制指令。

## 依赖项

- ROS 2 Humble 或更高版本
- Python 3.x
- `torch` (适用于 GraspNet 模型)
- `open3d` (点云处理)
- `cv_bridge` (图像转换)
- `sensor_msgs` 和 `geometry_msgs` 等 ROS 2 消息类型
- `graspnet` 模型和相关代码

## 安装

### 安装 ROS 2 和依赖项

1. 首先，确保你已安装 ROS 2 和相关的依赖项。可以参考 ROS 2 官方文档来安装 ROS 2。
   
2. 安装 Python 依赖项：

```bash
pip install torch open3d cv_bridge scipy
```

3. 克隆或下载本项目，并编译：

```bash
cd ~/ros2_ws/src
git clone https://your_repository_url/graspnet_generator.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 配置

### 模型检查点

请确保已下载 GraspNet 模型的检查点文件，并将其路径设置为配置文件中的 `--checkpoint_path` 参数。例如，下载检查点文件并存放到 `/path/to/checkpoint-rs.tar` 目录下。

```bash
# Example for configuration file
--checkpoint_path /path/to/checkpoint-rs.tar
```

### 配置参数

以下是主要的可配置参数：

- `--checkpoint_path`: GraspNet 模型的检查点路径（必需）。
- `--num_point`: 点云中的点数（默认为 `20000`）。
- `--num_view`: 视角数量（默认为 `300`）。
- `--collision_thresh`: 碰撞检测阈值（默认为 `0.01`）。
- `--voxel_size`: 点云处理的体素大小（默认为 `0.01`）。

### 目录结构

```
graspnet_generator/
├── config/                  # 配置文件
├── src/                     # 源代码
│   ├── graspnet_generator/
│   ├── graspnet_dataset/    # 数据集
│   ├── models/              # GraspNet 模型
│   ├── utils/               # 辅助工具
├── launch/                  # 启动文件
└── README.md                # 本文档
```

## 使用方法

### 启动 ROS 2 节点

在启动 `graspnet_generator` 节点之前，确保已经启动了 ROS 2 网络，并确保其他依赖节点（如图像发布者）已运行。

1. 启动 `graspnet_generator` 节点：

```bash
ros2 run graspnet_generator graspnet_generator
```

2. 启动其他相关节点（例如图像传感器和相机信息发布节点）。

### 服务调用

`graspnet_generator` 提供了一个 ROS 2 服务 `/get_grasp_poses`，用于请求抓取位姿。

#### 请求参数

- `calculate_grasp_poses`: 一个字符串参数，通常是 "calculate"，表示请求生成抓取位姿。

#### 响应参数

- `success`: 请求是否成功。
- `error_message`: 错误信息（如果有）。
- `poses`: 生成的抓取位姿列表。每个位姿为 `geometry_msgs/Pose` 类型，包含目标位置（`position`）和方向（`orientation`）。

### 例子

1. **请求抓取位姿服务**

   客户端可以通过 ROS 2 服务来请求抓取位姿。

```python
from grasp_get_interfaces.srv import GetPose
import rclpy
from rclpy.node import Node

class GraspPoseClient(Node):
    def __init__(self):
        super().__init__('grasp_pose_client')
        self.client = self.create_client(GetPose, '/get_grasp_poses')

    def request_grasp_poses(self):
        request = GetPose.Request()
        request.calculate_grasp_poses = "calculate"
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f'Received {len(response.poses)} grasp poses.')
        else:
            self.get_logger().error(f'Failed to get grasp poses: {response.error_message}')

def main(args=None):
    rclpy.init(args=args)
    client = GraspPoseClient()
    client.request_grasp_poses()
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

2. **监听任务指令**

   客户端可以监听上层大脑发布的任务指令，并根据指令（如 `pick up`）执行抓取任务。

```python
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class TaskListener(Node):
    def __init__(self):
        super().__init__('task_listener')
        self.create_subscription(String, '/task_command', self.listener_callback, 10)

    def listener_callback(self, msg):
        if "pick up" in msg.data:
            self.get_logger().info("Task received: Start pick up process")
            # Start requesting grasp poses or other actions

def main(args=None):
    rclpy.init(args=args)
    listener = TaskListener()
    rclpy.spin(listener)

if __name__ == '__main__':
    main()
```

## 测试

您可以通过调用 `/get_grasp_poses` 服务来测试抓取位姿生成是否正常工作。根据客户端的需求，您可以将生成的位姿发送到运动规划服务进行后续操作。

### 测试命令：

1. 在 ROS 2 中调用服务：

```bash
ros2 service call /get_grasp_poses grasp_get_interfaces/srv/GetPose "{calculate_grasp_poses: 'calculate'}"
```

