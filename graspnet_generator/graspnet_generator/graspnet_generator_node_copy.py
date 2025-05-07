"""基于GraspNet的抓取姿态生成ROS2节点模块。

该模块实现了一个ROS2节点，用于处理RGB-D相机数据并生成物体抓取姿态。
主要功能包括点云处理、碰撞检测和抓取姿态生成。

典型使用流程:
1. 订阅RGB、深度和掩码图像话题
2. 接收相机内参信息
3. 处理点云数据并分割工作区域
4. 使用预训练模型生成候选抓取姿态
5. 执行碰撞检测筛选有效抓取姿态
6. 通过服务接口返回抓取姿态结果

相关依赖:
- ROS2 Foxy Fitzroy
- PyTorch 1.8+
- Open3D 0.12+
"""

import os
import sys
import math
import termios
import tty
import copy
from grasp_get_interfaces.srv import GetPose

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import time
import cv2

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from image_geometry import PinholeCameraModel
from typing import Optional
from scipy.spatial.transform import Rotation as R

import numpy as np
import open3d as o3d
import argparse
import importlib
import scipy.io as scio
from PIL import Image as PILImage

import torch
from .utils.collision_detector import ModelFreeCollisionDetector
from .utils.data_utils import Camerainfo, create_point_cloud_from_depth_image
import gc



# 模型配置参数（根据Google Python风格指南使用全大写命名常量）
CHECKPOINT_PATH = '/home/konka/data/code/motion/src/graspnet_generator/graspnet_generator/checkpoint-rs.tar'
"""str: 预训练模型检查点文件路径。默认值指向项目内的检查点文件。"""

NUM_POINT = 20000
"""int: 点云处理时采样的点数。影响处理速度和精度，值越大精度越高但计算量越大。"""

NUM_VIEW = 300
"""int: 视角采样数量。用于生成不同视角下的抓取姿态候选。"""

COLLISION_THRESH = 0.01
"""float: 碰撞检测阈值（单位：米）。小于此值的碰撞距离将被视为有效碰撞。"""

VOXEL_SIZE = 0.01
"""float: 点云体素化处理的体素大小（单位：米）。用于降采样和加速碰撞检测。"""

# 初始化配置对象
cfgs = argparse.Namespace(
    checkpoint_path=CHECKPOINT_PATH,
    num_point=NUM_POINT,
    num_view=NUM_VIEW,
    collision_thresh=COLLISION_THRESH,
    voxel_size=VOXEL_SIZE
)


class GraspNetGenerator(Node):
    """基于GraspNet的抓取姿态生成ROS2节点。
    
    本节点负责处理RGB-D相机数据，生成物体抓取姿态，并通过ROS2服务接口返回结果。
    主要功能包括：
    - 订阅RGB、深度和掩码图像话题
    - 接收并处理相机内参
    - 点云预处理和有效区域分割
    - 使用预训练模型生成候选抓取姿态
    - 碰撞检测和姿态筛选
    - 坐标系转换和姿态发布

    Attributes:
        bridge (CvBridge): OpenCV与ROS2图像消息转换工具
        net (torch.nn.Module): 加载的预训练抓取生成模型
        image_subscription (rclpy.node.Subscription): RGB图像订阅器
        depth_subscription (rclpy.node.Subscription): 深度图像订阅器  
        mask_subscription (rclpy.node.Subscription): 掩码图像订阅器
        intrinsics (np.ndarray): 相机内参矩阵(3x3)
        tf_buffer (tf2_ros.Buffer): TF坐标变换缓冲区
        tf_listener (tf2_ros.TransformListener): TF坐标变换监听器
        rgb_image (Optional[np.ndarray]): 缓存的RGB图像数据(H,W,3)
        depth_image (Optional[np.ndarray]): 缓存的深度图像数据(H,W)
        mask_image (Optional[np.ndarray]): 缓存的掩码图像数据(H,W)
    """

    def __init__(self):
        """初始化抓取生成节点，创建ROS2组件并加载模型。

        初始化顺序:
            1. 调用父类构造函数创建节点
            2. 初始化CUDA内存缓存
            3. 创建OpenCV-ROS桥接器
            4. 加载预训练模型
            5. 创建各类订阅器和服务
            6. 初始化TF监听器
            7. 设置坐标系参数
            8. 初始化图像数据缓存

        属性:
            bridge (CvBridge): OpenCV与ROS2图像消息转换工具
            net (torch.nn.Module): 加载的预训练抓取生成模型
            image_subscription (Subscription): RGB图像订阅器
            depth_subscription (Subscription): 深度图像订阅器
            mask_subscription (Subscription): 掩码图像订阅器
            subscription (Subscription): 相机内参订阅器
            srv (Service): 抓取姿态请求服务
            tf_buffer (tf2_ros.Buffer): TF坐标变换缓冲区
            tf_listener (tf2_ros.TransformListener): TF坐标变换监听器
            camera_frame (str): 相机坐标系名称
            world_frame (str): 世界坐标系名称
            rgb_image (np.ndarray): 缓存的RGB图像数据
            depth_image (np.ndarray): 缓存的深度图像数据
            mask_image (np.ndarray): 缓存的掩码图像数据
            intrinsics (np.ndarray): 相机内参矩阵

        异常:
            RuntimeError: 当CUDA不可用或模型加载失败时抛出
        """
        super().__init__('graspnet_generator')
        torch.cuda.empty_cache()  # 清空CUDA缓存防止内存溢出

        # 初始化OpenCV-ROS桥接器
        self.bridge = CvBridge()

        # 创建图像订阅器（QoS深度设为10）
        self.image_subscription = self.create_subscription(
            Image,
            '/owl_output_image',
            self.rgb_image_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            '/owl_output_depth',
            self.depth_image_callback,
            10
        )

        self.mask_subscription = self.create_subscription(
            Image,
            '/owl_output_mask',
            self.mask_image_callback,
            10
        )

        # 创建相机内参订阅器
        self.subscription = self.create_subscription(
            CameraInfo,
            '/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10)

        # 初始化抓取姿态请求服务
        self.srv = self.create_service(
            GetPose, 
            '/get_grasp_poses', 
            self.handle_grasp_request
        )

        # 初始化TF2监听系统
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, 
            self,
            qos=10
        )

        # 设置坐标系参数
        self.camera_frame = 'head_camera_link'  # 相机坐标系名称
        self.world_frame = 'ground_link'        # 世界坐标系名称

        # 初始化图像数据缓存
        self.rgb_image = None    # 存储RGB图像数据 (H, W, 3)
        self.depth_image = None  # 存储深度图像数据 (H, W)
        self.mask_image = None   # 存储掩码图像数据 (H, W)
        self.intrinsics = None   # 相机内参矩阵 (3x3)

    def rgb_image_callback(self, msg: Image) -> None:
        """RGB图像订阅回调函数。
        
        处理流程:
            1. 将ROS图像消息转换为OpenCV格式
            2. 归一化像素值到[0,1]范围
            3. 存储到类属性供后续处理

        参数:
            msg (Image): ROS2 RGB图像消息，编码格式为bgr8

        副作用:
            更新self.rgb_image属性
        """
        self.rgb_image = self.process_image(msg, 'bgr8') / 255.0

    def depth_image_callback(self, msg: Image) -> None:
        """深度图像订阅回调函数。
        
        处理流程:
            1. 将ROS深度图像消息转换为numpy数组
            2. 保持原始深度值（单位：米）
            3. 存储到类属性供后续处理

        参数:
            msg (Image): ROS2深度图像消息，编码格式为passthrough

        副作用:
            更新self.depth_image属性
        """
        self.depth_image = self.process_image(msg, 'passthrough')

    def mask_image_callback(self, msg: Image) -> None:
        """掩码图像订阅回调函数。
        
        处理流程:
            1. 将ROS单通道图像消息转换为二值掩码
            2. 转换为布尔类型数组
            3. 存储到类属性供后续处理

        参数:
            msg (Image): ROS2单通道图像消息，编码格式为mono8

        副作用:
            更新self.mask_image属性
        """
        self.mask_image = self.process_image(msg, 'mono8')

    def process_image(self, msg: Image, encoding: str) -> Optional[np.ndarray]:
        """处理并转换ROS图像消息为OpenCV格式。

        参数:
            msg (Image): ROS2图像消息对象，包含原始图像数据
            encoding (str): 目标图像编码格式（如'bgr8', 'mono8'等）

        返回:
            Optional[np.ndarray]: 转换成功的numpy数组图像数据，转换失败返回None

        异常:
            无显式抛出异常，但会捕获转换错误并记录日志

        示例:
            >>> img = process_image(msg, 'bgr8')
            >>> print(img.shape)
            (480, 640, 3)
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, encoding)
            np_image = np.array(cv_image)
            return np_image
        except Exception as e:
            self.get_logger().error(f'图像格式转换失败: {e}')
            return None

    # owl_output_mask
    def camera_info_callback(self, msg):
        # Extract the camera matrix K from the CameraInfo message
        self.intrinsics = np.array(msg.k).reshape(3, 3)
        # Print or use the camera matrix K
        # self.get_logger().info(f'Camera Matrix K:{self.intrinsics}')

    def handle_grasp_request(
        self, 
        request: GetPose.Request,
        response: GetPose.Response
    ) -> GetPose.Response:
        """处理抓取姿态生成请求服务回调。

        Args:
            request (GetPose.Request): 服务请求对象，包含以下字段:
                - command (str): 控制命令（保留字段，当前版本未使用）
            response (GetPose.Response): 服务响应对象，包含以下字段:
                - success (bool): 请求处理是否成功
                - error_message (str): 错误描述（当success为False时）
                - poses (List[Pose]): 生成的抓取姿态列表（世界坐标系）

        Returns:
            GetPose.Response: 填充后的响应对象，包含:
                - success: True表示成功生成至少一个有效抓取姿态
                - error_message: 失败原因描述（success为False时有效）
                - poses: 有效抓取姿态列表（按质量降序排列）

        Raises:
            RuntimeError: 当遇到以下情况时通过响应返回错误:
                - 传感器数据未就绪（RGB/深度/掩码图像）
                - 相机内参未初始化
                - 点云处理失败
                - 坐标转换异常

        示例:
            >>> from grasp_get_interfaces.srv import GetPose
            >>> request = GetPose.Request()
            >>> response = handle_grasp_request(request, GetPose.Response())
            >>> assert isinstance(response, GetPose.Response)
        """
        self.get_logger().info('收到抓取姿态生成请求')
        if self.rgb_image is None:
            response.success = False
            response.error_message = "No RGB image received yet."
            return response
        if self.depth_image is None:
            response.success = False
            response.error_message = "No depth image received yet."
            return response
        if self.mask_image is None:
            response.success = False
            response.error_message = "No mask image received yet."
            return response
        if self.intrinsics is None:
            response.success = False
            response.error_message = "No camera info received yet."
            return response  # 相机参数未收到
        poses = self.process_data()
        print(poses)
        if poses is None:
            response.success = False
            response.error_message = "Failed to generate grasp poses."
            return response
        response.success = True
        response.error_message = ""
        response.poses = poses
        # response.marker = marker
        return response

    def process_data(self):
        """处理传感器数据并生成抓取姿态。

        实现流程:
            1. 数据有效性检查
            2. 点云预处理（降噪、滤波、分割）
            3. 抓取姿态生成模型推理
            4. 碰撞检测与姿态筛选
            5. 坐标系转换与姿态输出

        Args:
            self (GraspNetGenerator): 类实例，包含以下属性:
                rgb_image (np.ndarray): RGB图像数组，形状(H, W, 3)，取值范围[0.0, 1.0]
                depth_image (np.ndarray): 深度图像数组，形状(H, W)，单位米
                mask_image (np.ndarray): 工作区域掩码图像，形状(H, W)，布尔类型
                intrinsics (np.ndarray): 相机内参矩阵(3x3)

        Returns:
            Optional[List[Pose]]: 有效抓取姿态列表，包含以下情况:
                - 成功生成姿态: 返回Pose对象列表
                - 数据不完整/处理失败: 返回None

        Raises:
            ValueError: 当传感器数据格式不正确时
            RuntimeError: 当点云处理或坐标转换失败时

        示例:
            >>> node = GraspNetGenerator()
            >>> poses = node.process_data()
            >>> assert isinstance(poses, list) or poses is None
        """
        """从类属性获取传感器数据并进行基本验证

        参数:
            self.rgb_image (np.ndarray): RGB图像数组，形状(H, W, 3)，取值范围[0.0, 1.0]
            self.depth_image (np.ndarray): 深度图像数组，形状(H, W)，单位米
            self.mask_image (np.ndarray): 工作区域掩码图像，形状(H, W)，布尔类型

        返回:
            tuple: 包含以下元素的元组:
                color (np.ndarray): 预处理后的RGB图像数据
                depth (np.ndarray): 深度图像数据
                workspace_mask (np.ndarray): 有效工作区域掩码

        异常:
            ValueError: 当图像数据不满足预期形状或数值范围时抛出
        """
        if self.rgb_image is None or self.depth_image is None or self.mask_image is None or self.intrinsics is None:
            return
        color = self.rgb_image
        depth = self.depth_image

        # 获取并处理工作区域掩码
        workspace_mask = self.mask_image
        # workspace_mask = np.array(PILImage.open(os.path.join('/home/ubuntu/ros2_ws/src/ros2_graspnet/ros2_graspnet/', 'mask_image.png')))
        height, width = color.shape[:2]
        factor_depth = 1000.0
        camera = Camerainfo(width, height, self.intrinsics[1][1], self.intrinsics[0][0],
                            self.intrinsics[1][2], self.intrinsics[0][2], factor_depth)
  

        #点云滤波
        def pcd_ground_seg_open3d(scan: o3d.geometry.PointCloud, 
                                config: dict) -> tuple:
            """使用RANSAC算法进行地面点云分割。
            
            通过平面拟合方法分离地面点云和非地面点云，并对地面点云进行颜色标注。

            Args:
                scan (o3d.geometry.PointCloud): 输入点云对象
                config (dict): 配置参数字典，应包含:
                    - ground_color (list): 地面点云RGB颜色值，范围[0,1]
                    - rest_color (list): 非地面点云RGB颜色值，范围[0,1]（当前未使用）

            Returns:
                tuple: 包含两个点云对象的元组:
                    - ground (o3d.geometry.PointCloud): 地面点云（已着色）
                    - rest (o3d.geometry.PointCloud): 非地面点云

            Raises:
                ValueError: 当输入点云为空或配置参数不完整时抛出
                
            实现步骤:
                1. 深拷贝原始点云防止数据污染
                2. 使用RANSAC算法进行平面分割
                3. 根据分割结果提取地面和非地面点云
                4. 对地面点云进行颜色标注
            """
            if not scan.has_points():
                raise ValueError("输入点云为空")
                
            if 'ground_color' not in config:
                raise ValueError("配置参数缺少ground_color")

            pcd = copy.deepcopy(scan)

            # 使用RANSAC算法进行平面分割（参数根据实际场景调整）
            ground_model, ground_indexes = scan.segment_plane(
                distance_threshold=0.018,  # 平面距离阈值（单位：米）
                ransac_n=3,               # 每次迭代采样点数
                num_iterations=1000       # RANSAC迭代次数
            )

            # 获取地面点云（反选索引）
            ground = pcd.select_by_index(ground_indexes, invert=True)
            
            # 获取非地面点云
            rest = pcd.select_by_index(ground_indexes)

            # 可视化标注：地面点云着色
            ground.paint_uniform_color(config['ground_color'])

            return ground, rest

        # 配置颜色
        config = {
            'ground_color': [0.8, 0.2, 0.2],  # 红色表示地面
            'rest_color': [0.2, 0.8, 0.2]  # 绿色表示剩余点云
        }

        # 假设 depth 和 color 已经定义
        cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)
        # print("cloud的大小:", cloud.shape)

        # 假设 workspace_mask 已经定义
        mask = (workspace_mask & (depth > 0))
        mask = mask.astype(bool)
        # print("应用 mask 前点云数量:", cloud.shape[0])

        cloud_masked = cloud[mask]
        color_masked = color[mask]
        # print("应用 mask 后点云数量:", cloud_masked.shape[0])

        # 创建 Open3D 点云对象，用于处理
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
        pcd.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))

        # 使用 Open3D 进行平面分割
        ground, rest = pcd_ground_seg_open3d(pcd, config)

        # 可视化地面和剩余的点云
        # o3d.visualization.draw_geometries([ground, rest])

        # 对剩余的点云进行离群点去除
        cl, index = rest.remove_statistical_outlier(nb_neighbors=50, std_ratio=2.0)
        filtered_rest = rest.select_by_index(index)

        # 将滤波后的点云转换为 numpy 数组
        cloud_filtered = np.asarray(filtered_rest.points)
        color_filtered = np.asarray(filtered_rest.colors)

        # 测试
        # 找到点云中最靠前的点（假设 Z 坐标为深度值）
        min_depth_index = np.argmin(cloud_filtered[:, 2])  # 找到 Z 坐标最小的点的索引
        origin = cloud_filtered[min_depth_index]  # 使用该点作为物体参考点
        # print("物体参考点（最靠前的点）:", origin)

        # 计算每个点到参考点的欧几里得距离
        distances = np.linalg.norm(cloud_filtered - origin, axis=1)

        # 筛选距离小于等于 5 cm 的点
        distance_threshold = 0.2  # 距离阈值 5 cm
        valid_indices = distances <= distance_threshold

        # 筛选符合条件的点和颜色
        cloud_final = cloud_filtered[valid_indices]
        color_final = color_filtered[valid_indices]

        # 找到点云的几何中心（质心）
        centroid = np.mean(cloud_final, axis=0)
        print("点云质心（几何中心）:", centroid)

        # 找到最接近质心的点
        distances_to_centroid = np.linalg.norm(cloud_final - centroid, axis=1)
        center_point_index = np.argmin(distances_to_centroid)
        center_point = cloud_final[center_point_index]
        print("最中间的点位置:", center_point)

        grasp_target = np.array([
            [0.998, 0.058, 0.021, center_point[0]],
            [0.056, -0.730, -0.681, center_point[1]],
            [-0.024, 0.681, -0.732, center_point[2]],
            [0.000, 0.000, 0.000, 1.000]
        ])
        grasp_pose = self.process_model_output(grasp_target)
        grasp_poses = []
        grasp_poses.append(grasp_pose)  # 仅添加有效的抓取姿态
        grasp_poses.append(grasp_pose) 
        
        # cube_marker = Marker()
        # cube_marker.header.frame_id = "ground_link"  # 替换为实际的坐标系
        # cube_marker.header.stamp = self.get_clock().now().to_msg()
        # cube_marker.ns = "bounding_box"
        # cube_marker.id = 1
        # cube_marker.type = Marker.CUBE
        # cube_marker.action = Marker.ADD
        # c_box_max = np.array([
        #     [0.998, 0.058, 0.021, max_bound[0]],
        #     [0.056, -0.730, -0.681, max_bound[1]],
        #     [-0.024, 0.681, -0.732, max_bound[2]],
        #     [0.000, 0.000, 0.000, 1.000]
        # ])
        # w_box_max = self.process_model_output(c_box_max)
        # c_box_min = np.array([
        #     [0.998, 0.058, 0.021, min_bound[0]],
        #     [0.056, -0.730, -0.681, min_bound[1]],
        #     [-0.024, 0.681, -0.732, min_bound[2]],
        #     [0.000, 0.000, 0.000, 1.000]
        # ])
        # w_box_min = self.process_model_output(c_box_min)
        # cube_marker.pose.position.x = (w_box_min.position.x + w_box_max.position.x) / 2
        # cube_marker.pose.position.y = (w_box_min.position.y + w_box_max.position.y) / 2
        # cube_marker.pose.position.z = (w_box_min.position.z + w_box_max.position.z) / 2 +0.05
        # print("cube_marker.pose.position:", cube_marker.pose.position)
        # cube_marker.scale.x = w_box_max.position.x - w_box_min.position.x
        # cube_marker.scale.z = w_box_max.position.z - w_box_min.position.z
        # cube_marker.scale.y = 0.2 # 自定义深度
        # cube_marker.color.a = 0.5  # 半透明
        # cube_marker.color.r = 0.0
        # cube_marker.color.g = 1.0
        # cube_marker.color.b = 0.0
        return grasp_poses

    def process_model_output(self, output: np.ndarray) -> Pose:
        """将模型输出的4x4变换矩阵转换为ROS Pose消息。

        实现步骤:
            1. 定义相机到机械臂基座的标定矩阵
            2. 执行坐标系变换: world_pose = T_base_camera * model_output
            3. 提取平移分量并做机械臂特定的偏移补偿
            4. 设置默认姿态（朝上的垂直抓取）

        参数:
            output (np.ndarray): 模型输出的4x4齐次变换矩阵，形状(4,4)
                矩阵结构:
                [
                    [r11, r12, r13, tx],
                    [r21, r22, r23, ty],
                    [r31, r32, r33, tz],
                    [  0,   0,   0,  1]
                ]
                其中:
                - rij: 旋转矩阵元素
                - tx, ty, tz: 平移分量（米）

        返回:
            Pose: ROS姿态消息，包含:
                - position: 三维坐标（世界坐标系）
                - orientation: 四元数（默认朝上的垂直姿态）

        异常:
            ValueError: 当输入矩阵形状不是4x4时抛出
            TypeError: 当输入不是numpy数组时抛出

        数学公式:
            world_pose = T_base_camera × model_output
            其中:
            - T_base_camera: 机械臂基座到相机的标定矩阵
            - model_output: 模型输出的抓取姿态矩阵
        """
        # 验证输入矩阵形状
        if output.shape != (4, 4):
            raise ValueError(f"无效的变换矩阵形状: {output.shape}，应为(4,4)")
            
        # 定义标定矩阵（相机到机械臂基座）
        # trans_matrix = np.array([
        #     [-0.059, -0.793, 0.607, 0.047],   # X轴方向分量
        #     [-0.988, 0.135, 0.080, 0.023],    # Y轴方向分量
        #     [-0.145, -0.594, -0.791, 1.441],  # Z轴方向分量
        #     [0.000, 0.000, 0.000, 1.000]      # 齐次坐标补位
        # ])

        tf_msg = self.tf_buffer.lookup_transform('camera_color_optical_frame', 'base_link', rclpy.time.Time())
        # 提取平移向量
        translation = [
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z
        ]
        # 提取旋转四元数
        rotation = [
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w
        ]
        # 将旋转四元数转换为 3x3 旋转矩阵
        trans_matrix = tf_transformations.quaternion_matrix(rotation)
        # 将平移向量添加到旋转矩阵中，得到 4x4 齐次变换矩阵
        trans_matrix[:3, 3] = translation

        poseout = Pose()  # 初始化ROS Pose对象

        # 执行坐标系转换：将模型输出从相机坐标系转换到世界坐标系
        pose_out_matrix = np.dot(trans_matrix, output)  # 矩阵乘法实现坐标系变换
        
        # 设置位置坐标并进行机械臂末端执行器的偏移补偿
        # 注：偏移量根据实际机械臂校准结果确定
        # poseout.position.x = pose_out_matrix[0, 3] - 0.12  # X轴补偿（单位：米）
        # poseout.position.y = pose_out_matrix[1, 3] + 0.04  # Y轴补偿（单位：米）
        # poseout.position.z = pose_out_matrix[2, 3] - 0.17  # Z轴补偿（单位：米）
        poseout.position.x = pose_out_matrix[0, 3]  # X轴补偿（单位：米）
        poseout.position.y = pose_out_matrix[1, 3]  # Y轴补偿（单位：米）
        poseout.position.z = pose_out_matrix[2, 3]  # Z轴补偿（单位：米）

        # 设置默认四元数（垂直向下抓取姿态）
        # 四元数格式: (x, y, z, w)
        poseout.orientation.x = 0.0  # 绕X轴旋转分量
        poseout.orientation.y = 0.0  # 绕Y轴旋转分量
        poseout.orientation.z = 0.0  # 绕Z轴旋转分量
        poseout.orientation.w = 1.0  # 标量分量（无旋转）

        return poseout

    def transform_to_matrix(self, transform: TransformStamped) -> np.ndarray:
        # 提取平移
        translation = transform.transform.translation
        translation_vector = np.array([
            translation.x,
            translation.y,
            translation.z
        ])

        # 提取旋转
        rotation = transform.transform.rotation
        quaternion = np.array([
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w
        ])

        # 生成 4x4 矩阵
        matrix = np.eye(4)
        matrix[0:3, 3] = translation_vector
        rotation_matrix = R.from_quat(quaternion).as_matrix()
        matrix[0:3, 0:3] = rotation_matrix

        return matrix


def main(args=None):
    """ROS2节点的主入口函数。

    实现流程:
        1. 初始化ROS2客户端库
        2. 创建抓取生成节点实例
        3. 启动节点事件循环
        4. 清理节点资源
        5. 关闭ROS2客户端

    参数:
        args (List[str], optional): 命令行参数列表。默认为None。

    返回:
        None

    示例:
        >>> main()
        # 启动抓取生成节点并进入事件循环
    """
    rclpy.init(args=args)  # 初始化ROS2上下文
    try:
        node = GraspNetGenerator()  # 创建节点实例
        rclpy.spin(node)  # 保持节点运行直到收到终止信号
    except KeyboardInterrupt:
        pass  # 处理键盘中断信号
    finally:
        # 清理资源
        node.destroy_node()  # 销毁节点
        rclpy.try_shutdown()  # 安全关闭ROS2上下文

if __name__ == '__main__':
    main()
