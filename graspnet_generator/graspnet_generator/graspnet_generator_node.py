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
import tf_transformations

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
from PIL import ImageDraw, ImageFont
from .utils.data_utils import Camerainfo, create_point_cloud_from_depth_image
import json

import torch

import gc


class GraspNetGenerator(Node):
    def __init__(self):
        super().__init__('graspnet_generator')
        torch.cuda.empty_cache()

        # # 创建CvBridge实例
        self.bridge = CvBridge()
        # # 订阅RGB图像
        # self.image_subscription = self.create_subscription(
        #     Image,
        #     '/owl_output_image',
        #     self.rgb_image_callback,
        #     10
        # )

        # # 订阅深度图像
        # self.depth_subscription = self.create_subscription(
        #     Image,
        #     '/owl_output_depth',
        #     self.depth_image_callback,
        #     10
        # )

        # # 订阅掩码图像
        # self.mask_subscription = self.create_subscription(
        #     Image,
        #     '/owl_output_mask',
        #     self.mask_image_callback,
        #     10
        # )
        # 订阅相机信息
        self.subscription = self.create_subscription(
            CameraInfo,
            '/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10)

        # 创建服务：用于请求抓取位姿
        self.srv = self.create_service(GetPose, '/get_grasp_poses', self.handle_grasp_request)

        # TF2监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_frame = 'head_camera_link'
        self.world_frame = 'ground_link'

        self.rgb_image = None
        self.depth_image = None
        self.mask_image = None
        self.intrinsics = None

    def rgb_image_callback(self, msg):
        self.rgb_image = self.process_image(msg, 'bgr8') / 255.0

    def depth_image_callback(self, msg):
        self.depth_image = self.process_image(msg, 'passthrough')

    def mask_image_callback(self, msg):
        self.mask_image = self.process_image(msg, 'mono8')

    def process_image(self, msg, encoding):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, encoding)
            np_image = np.array(cv_image)
            return np_image
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
            return None
    
    def plot_boxes_to_image(self, size, detect_2d):
        rect = detect_2d['left_top'] + detect_2d['right_bottom']
        #生成灰度图
        mask = PILImage.new("L", size, color='black')
        mask_draw = ImageDraw.Draw(mask)

        mask_draw.rectangle(rect, fill='white')

        return mask

    # owl_output_mask
    def camera_info_callback(self, msg):
        # Extract the camera matrix K from the CameraInfo message
        self.intrinsics = np.array(msg.k).reshape(3, 3)

    def handle_grasp_request(self, request, response):
        """服务回调，处理抓取请求"""
        cmd_info = json.loads(request.cmd_json)
        w,h = cv2.imread(cmd_info.get('image_rgb_url')).shape[:2]
        size = [h,w]
        self.rgb_image   = np.array(cv2.imread(cmd_info.get('image_rgb_url'))) / 255.0
        self.depth_image = np.array(cv2.imread(cmd_info.get('image_depth_url'), cv2.IMREAD_UNCHANGED), dtype=np.float32)

        detection_2d = cmd_info.get('detection_2d')
        
        self.mask_image = self.plot_boxes_to_image(size, detection_2d)
        self.mask_image = np.array(self.mask_image)

        self.get_logger().info('Received request for grasp poses.')
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
        if self.rgb_image is None or self.depth_image is None or self.mask_image is None or self.intrinsics is None:
            return
        color = self.rgb_image
        depth = self.depth_image
        print("depth:")
        print(depth.shape)
        # adjust camera para
        matrix = [0.0, 606.0, 246.0,
                  -606.0, 0.0, 321.0,
                  0.0, 0.0, 1.0]
        self.intrinsics = np.array([matrix]).reshape(3,3)
        #self.intrinsics = np.array([self.intrinsics]).reshape(3,3)
        print("intrinsics:",self.intrinsics)
        # input("Press Enter to continue...")
        workspace_mask = self.mask_image
        # workspace_mask = np.array(PILImage.open(os.path.join('/home/ubuntu/ros2_ws/src/ros2_graspnet/ros2_graspnet/', 'mask_image.png')))
        height, width = color.shape[:2]
        factor_depth = 1000.0
        # camera = Camerainfo(width, height, self.intrinsics[1][1], self.intrinsics[0][0],
        #                     self.intrinsics[1][2], self.intrinsics[0][2], factor_depth)
        camera = Camerainfo(width, height, self.intrinsics, factor_depth)
        print(camera.scale)

        #点云滤波
        def pcd_ground_seg_open3d(scan, config):
            """ 使用 Open3D 的 RANSAC 算法进行平面分割 """
            pcd = copy.deepcopy(scan)

            # 平面分割，distance_threshold 设置为分割时的距离阈值
            ground_model, ground_indexes = scan.segment_plane(distance_threshold=0.018,
                                                              ransac_n=3,
                                                              num_iterations=1000)
            ground_indexes = np.array(ground_indexes)


            # 剩余的点云（非平面）
            rest= pcd.select_by_index(ground_indexes)
            # 分割后的地面点云
            ground = pcd.select_by_index(ground_indexes, invert=True)

            # # 给地面和非地面点云上色，方便可视化
            ground.paint_uniform_color(config['ground_color'])  # 地面点云颜色
            # # rest.paint_uniform_color(config['rest_color'])  # 剩余点云颜色

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

            # 创建表示中心点的红色球体（半径可根据点云尺度调整）
        center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
        center_sphere.paint_uniform_color([1, 0, 0])  # 红色
        center_sphere.translate(center_point)  # 将球体移动到中心点位置

        # 将最终筛选后的点云转换为Open3D点云对象
        final_pcd = o3d.geometry.PointCloud()
        final_pcd.points = o3d.utility.Vector3dVector(cloud_final)
        final_pcd.colors = o3d.utility.Vector3dVector(color_final)

        # 可视化地面、最终点云和中心点标记
        o3d.visualization.draw_geometries([ground, final_pcd, center_sphere])

        # grasp_target = np.array([
        #     [0.998, 0.058, 0.021, center_point[0]],
        #     [0.056, -0.730, -0.681, center_point[1]],
        #     [-0.024, 0.681, -0.732, center_point[2]],
        #     [0.000, 0.000, 0.000, 1.000]
        # ])
        grasp_target = np.array([
            [1.000, 0.000, 0.000, center_point[0]],
            [0.000, 1.000, 0.000, center_point[1]],
            [0.000, 0.000, 1.000, center_point[2]],
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





    def process_model_output(self, output):
        # 处理模型输出并获取第一个位姿
        poseout = Pose()
        # trans_matrix = np.array([
        #     [-0.059, -0.793, 0.607, 0.047],
        #     [-0.988, 0.135, 0.080, 0.023],
        #     [-0.145, -0.594, -0.791, 1.441],
        #     [0.000, 0.000, 0.000, 1.000]
        # ])

        tf_msg = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rclpy.time.Time())
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
        print("matrix:", trans_matrix)
        pose_out_matrix = np.dot( trans_matrix, output)
        poseout.position.x = pose_out_matrix[0, 3] + 0.02
        poseout.position.y = pose_out_matrix[1, 3]
        poseout.position.z = pose_out_matrix[2, 3] 
        print(poseout.position)
        poseout.orientation.x = 0.0
        poseout.orientation.y = 0.0
        poseout.orientation.z = 0.0
        poseout.orientation.w = 1.0

        return poseout

def main(args=None):
    rclpy.init(args=args)
    node = GraspNetGenerator()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
