# gongga_ros2

该仓库包含二维码识别及其tf广播，贡嘎机器人的驱动，用于配合机器人urdf标定、相机标定、和上层规划器任务。

# 1 安装

1. 安装贡嘎机器人电机驱动及其接口，参考：

   ```
   https://github.com/Humanoid-Robot-Innovation-Center/motor_driver
   https://github.com/Humanoid-Robot-Innovation-Center/gongga_interface
   ```

2. 安装real_sense ros功能包及相关依赖

   ```
   sudo apt-get install ros-humble-realsense2-camera
   sudo apt install ros-humble-tf-transformations
   ```

3. 编译

   ```
   colcon build --packages-select gongga_core
   ```

   

# 2 使用

## 2.1 二维码检测

二维码检测通过检测二维码，获取二维码在相机坐标中的位置及姿态实现相机标定和后期通过粘贴二维码实现目标物体抓取及操作的功能拓展。detact_aruco_markers获取real_sense发布的RGB图和深度图，向/tf中广播检测到的二维码与相机坐标的坐标转换及rviz2中的可视化信息。

1. 生成自定义二维码：https://chev.me/arucogen/

2. 在/gongga_core/config/ray_marker_dict.yaml中配置自定义二维码的尺寸，对应link或物体的名称

3. 启动real_sense节点（通过ros2 topic list检查rgb图和深度图的话题名，如果与代码中的订阅不同需要修改代码）

   ```
   ros2 launch realsense2_camera rs_launch.py
   ```

4. 启动二维码检测节点

   ```
   ros2 launch ray_core ray_aruco.launch.py
   ```

## 2.2 gongga_driver

gongga_driver用于接收上层发送的请求并通过motor_interface驱动机器人，目前仅实现了开环的关节轨迹执行。

gongga_driver通过**ExecuteTrajectory**服务接收外部发送的关节轨迹:

1. 请求和响应

   request  int32:control_rate, JointState:trajectory # 待执行的轨迹

   response bool:is_success, str:failure_reason # 轨迹是否执行成功及相关信息

   

   

   