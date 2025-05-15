# Grasp Object Client 

## 1. **项目简介**

`grasp_object_client` 是一个 ROS 2 客户端节点，负责在接收到上层大脑发布的任务指令后，自动执行抓取任务。客户端会订阅一个上层大脑发布的任务指令话题（如 `/brain/commands`），当接收到包含 "pick up" 关键词的指令时，客户端会执行以下步骤：

1. 向 `graspnet_generator` 服务请求抓取位姿。
2. 向 `motion_generator` 服务发送抓取位姿进行运动轨迹规划。
3. 向下层驱动服务发送关节轨迹，控制机器人完成抓取任务。

## 2. **功能概述**

- **等待任务指令**：客户端会订阅上层大脑发布的 `/brain/commands` 话题，等待任务指令。
- **检测关键词**：当接收到包含 "pick up" 关键词的指令时，开始执行抓取任务。
- **请求抓取位姿**：客户端向 `graspnet_generator` 服务请求多个抓取位姿。
- **发送位姿进行运动规划**：客户端将抓取位姿传递给 `motion_generator` 服务进行轨迹规划。
- **发送关节轨迹**：客户端将运动规划后的关节轨迹发送给下层 `gongga_driver` 服务进行执行。
- **发布者**：向`/brain/feedback`发送关于任务状态的反馈。

## 3. **依赖项**

- ROS 2（版本：Humble 或更新版本）
- Python 3.10 或更新版本
- ROS 2 包：
  - `grasp_get_interfaces`（提供抓取位姿服务）
  - `gongga_interface`（提供运动规划服务）
  - `sensor_msgs`（用于处理 `JointState` 和 `Pose` 消息）
  - `std_srvs`（用于与 `gongga_driver` 进行通信）

## 4. **安装和设置**

### 4.1 克隆并构建工作空间

在 ROS 2 工作空间中，执行以下命令来克隆并构建所需的包：

```bash
cd ~/ros2_ws/src
git clone <graspnet_generator_repository_url>   # 克隆抓取位姿生成包
git clone <motion_generator_repository_url>      # 克隆运动规划包
cd ~/ros2_ws
colcon build
```

### 4.2 安装依赖包

确保安装了必要的 ROS 2 包：

```bash
sudo apt install ros-humble-grasp-get-interfaces
sudo apt install ros-humble-motion-generator-interfaces
```

### 4.3 配置

在运行客户端之前，确保以下服务已启动，并且可以正常通信：

1. `graspnet_generator` 服务：生成抓取位姿。
2. `motion_generator` 服务：生成运动规划。
3. `gongga_driver` 服务：控制机器人执行轨迹。

## 5. **如何使用**

### 5.1 启动 ROS 2 网络

首先，启动 ROS 2 网络，确保 ROS 2 Master 已经运行：

```bash
source ~/ros2_ws/install/setup.bash
```

### 5.2 启动上层大脑发布任务指令

上层大脑节点会定期发布任务指令到 `/brain/commands` 话题，客户端会订阅该话题。上层大脑发布消息的代码示例：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BrainPublisher(Node):
    def __init__(self):
        super().__init__('brain_publisher')
        self.publisher_ = self.create_publisher(String, '/brain/commands', 10)
        timer_period = 2  # 2 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 发布任务指令
        msg = String()
        msg.data = 'pick up'  # 任务指令
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published task command: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    brain_publisher = BrainPublisher()
    rclpy.spin(brain_publisher)

if __name__ == '__main__':
    main()
```

### 5.3 启动客户端

客户端会订阅 `/brain/commands` 话题并根据接收到的指令执行任务。启动客户端代码：

```bash
ros2 run graspnet_generator grasp_object_client
```

### 5.4 客户端执行流程

1. **监听任务指令**：客户端订阅 `/brain/commands` 话题，等待上层大脑发布任务指令。
2. **检测指令**：当客户端接收到 `pick up` 指令时，它会触发任务执行流程。
3. **请求抓取位姿**：客户端向 `graspnet_generator` 服务请求抓取位姿。
4. **规划运动轨迹**：客户端将抓取位姿传递给 `motion_generator` 服务进行运动轨迹规划。
5. **发送关节轨迹**：客户端将规划后的关节轨迹发送给 `gongga_driver` 服务，控制机器人执行任务。
6. **反馈**：通过`/brain/feedback`提供有关任务状态的连续反馈。

### 5.5 查看日志和调试

客户端在运行时会输出日志信息，您可以根据输出日志查看执行情况，例如：

```
[INFO] [grasp_object_client]: Waiting for task command from upper brain...
[INFO] [grasp_object_client]: Received task command: "pick up"
[INFO] [grasp_object_client]: Requesting grasp poses from graspnet_generator...
[INFO] [grasp_object_client]: Received 5 grasp poses.
[INFO] [grasp_object_client]: Sending grasp pose 1 to motion_generator...
[ERROR] [grasp_object_client]: Motion planning failed for grasp pose 1. Reason: Insufficient space.
[INFO] [grasp_object_client]: Sending grasp pose 2 to motion_generator...
[INFO] [grasp_object_client]: Motion planning succeeded for grasp pose 2.
```

### 5.6 上层大脑与客户端交互

上层大脑通过发布字符串消息到 `/brain/commands` 话题，向客户端发送任务指令。客户端通过监听该话题并解析其中的任务指令，执行相应的抓取任务。

上层大脑发布的消息格式为：

```python
# 上层大脑发布指令
String task_name  # 任务指令，例如 "pick up"
```

客户端会订阅 `/brain/commands` 话题，解析指令内容，当检测到 `pick up` 关键词时，开始执行任务。
