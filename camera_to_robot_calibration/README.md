# Camera_to_robot_calibration

## 简介

该ROS 2节点旨在从相机坐标系中获取多个marker的变换信息，并计算相机相对于机器人基坐标系的变换。通过处理从相机到各个marker的变换矩阵，该节点能够过滤异常值并使用Karcher均值算法计算平均变换。

## 功能

* 从预定义的marker ID列表中查询每个marker到相机的变换。
* 将相机到marker的变换与预定义的marker到基坐标系的变换结合，计算相机到基坐标系的变换。
* 处理异常值并使用Karcher均值算法计算变换均值。
* 定期发布相机到基坐标系的变换。

## 运行

1. 确保启动相机节点和marker检测节点。
2. 运行Camera to Base转换节点：

    ```bash
    ros2 run camera_to_robot_calibration calibration_node
    ```
3. 查看发布的变换：

    ```bash
    ros2 topic echo /tf
    ```

## 配置参数

## 配置参数

* ​`marker_ids`​: 要跟踪的marker ID列表，默认值为 `[1, 2, 3]`​。
* ​`transform_count`​: 要收集的变换矩阵数量，默认值为 `100`​。
* ​`base_to_marker_transforms`​: 预定义的从基坐标系到每个marker的旋转矩阵。需要在代码中设置每个marker的变换矩阵，例如：

  ```python
  self.base_to_marker_transforms = {
      1: np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]]),  # 替换为实际的变换矩阵
      2: np.array([[0, -1, 0, 0],
                   [1, 0, 0, 1],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]]),  # 替换为实际的变换矩阵
      3: np.array([[1, 0, 0, 1],
                   [0, 1, 0, 0],
                   [0, 0, 1, 2],
                   [0, 0, 0, 1]])   # 替换为实际的变换矩阵
  }
  ```

‍
