#######################################################################
#本文件用于贡嘎1号机器人仿真运行，可以通过ros2和实际工程文件进行交互
#功能：  使用ROS2和实际工程文件进行交互；
#       在仿真中验证motion模块的功能； 
#使用方法： 首先运行本文件，isaac_sim准备完毕后运行工程文件；
#编写人：蒋沛言
#时间：2025-4-23
#######################################################################

# 导入isaac sim接口
try:
    import isaacsim
except ImportError:
    pass
from omni.isaac.kit import SimulationApp
# 此处必须先实例化，否则后续无法导入模块
simulation_app = SimulationApp(
    {
        "headless": None,
        "width": "1920",
        "height": "1080",
    }
)
simulation_app.update()
from .helper import add_robot_to_scene
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.debug_draw import _debug_draw

# 导入ros2相关库
import rclpy
from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from motor_interface.msg import VelAndPose  # 自定义消息类型，用于控制电机
from motor_interface.msg import DebugData
from motor_interface.srv import GetJointState
# 导入curobo相关库
from curobo.util.usd_helper import UsdHelper
from curobo.geom.types import WorldConfig
from curobo.util_file import (
    load_yaml,
)
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.types.camera import CameraObservation
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.geom.types import Cuboid
# 其他库
import numpy as np
from functools import partial
import sys
import os
from ament_index_python.packages import get_package_share_directory
from .my_helper import *
import time
import json
import cv2
import torch
from matplotlib import cm

a = torch.zeros(4, device="cuda:0")

class simulator(Node):
# 同时运行仿真平台和ros2节点
# 用于模拟并反馈机器人运动
    def __init__(self):
        super().__init__('simulation_node')
        self.ros2_init()
        self.isaacsim_init()
    
    def ros2_init(self):
    # 初始化ros2节点
        self.base_z_pos_subscriber = self.create_subscription(
                                        VelAndPose, "base_z_pos", self.base_z_pos_callback, 100)
        self.base_z_vel_subscriber = self.create_subscription(
                                        VelAndPose, "base_z_vel", self.base_z_vel_callback, 100)
        self.wrist_yaw_subscriber = self.create_subscription(
                                        VelAndPose, "wrist_yaw", self.wrist_yaw_callback, 100)
        self.wrist_pitch_subscriber = self.create_subscription(
                                        VelAndPose, "wrist_pitch", self.wrist_pitch_callback, 100)
        self.wrist_roll_subscriber = self.create_subscription(
                                        VelAndPose, "wrist_roll", self.wrist_roll_callback, 100)
        self.arm_subscriber = self.create_subscription(
                                        VelAndPose, "cmd_arm", self.cmd_arm_callback, 100)
        self.lift_subscriber = self.create_subscription(
                                        VelAndPose, "cmd_lift", self.cmd_lift_callback, 100)
        self.gripper_subscriber = self.create_subscription(
                                        VelAndPose, "gripper_pull", self.gripper_pull_callback, 100)
        # 订阅运动轨迹，用于可视化
        self.debugSubscriber = self.create_subscription(
                                        DebugData, '/brain/ee_pos', self.debug_callback, 100)

        #电机状态反馈服务
        self.jointStateSrv = self.create_service(GetJointState, '/get_joint_state', self.jointStateSrv_callback)
        # 心跳检测定时器
        self.heartbeat_subscription = self.create_subscription(String, '/brain/heartbeat', self.heartbeat_callback, 10)
        # 设定心跳时间限制
        self.heartbeat_received = True
        self.heartbeat_timer = self.create_timer(1.0, self.check_heartbeat)

    def base_z_pos_callback(self, msg):
        _, _, yaw = quaternion_to_euler(msg.pose.orientation.w,
                                                msg.pose.orientation.x,
                                                msg.pose.orientation.y,
                                                msg.pose.orientation.z)
        # 底盘为相对运动模式
        self.cmd_motor_pos[0] += yaw
        # print("[debug]:yaw  = ", yaw)
    def base_z_vel_callback(self, msg):
        self.cmd_motor_vel[0] = msg.vel.angular.z
    def wrist_yaw_callback(self, msg):
        _, _, yaw = quaternion_to_euler(msg.pose.orientation.w,
                                        msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z)
        self.cmd_motor_pos[5] = yaw
    def wrist_pitch_callback(self, msg):
        _, _, yaw = quaternion_to_euler(msg.pose.orientation.w,
                                        msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z)
        self.cmd_motor_pos[6] = yaw
    def wrist_roll_callback(self, msg):
        _, _, yaw = quaternion_to_euler(msg.pose.orientation.w,
                                        msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z)
        self.cmd_motor_pos[7] = yaw
    def cmd_arm_callback(self, msg):
        # l2
        self.cmd_motor_pos[2] = msg.pose.position.x/3
        # l1
        self.cmd_motor_pos[3] = msg.pose.position.x/3
        # l0
        self.cmd_motor_pos[4] = msg.pose.position.x/3
    def cmd_lift_callback(self, msg):
        self.cmd_motor_pos[1] = msg.pose.position.z
    def gripper_pull_callback(self, msg):
        pass

    
    def jointStateSrv_callback(self, request, response):
    # 电机状态反馈服务回调函数
        sim_js_states = self.robot.get_joints_state()
        if sim_js_states is None:
            print("无法获取仿真机器人状态，可能是舞台未启动")
            return response
        
        sim_js_names = self.robot.dof_names
        # 选择需要反馈的关节
        fbk_name = [sim_js_names[idx] for idx in self.idx_list]
        fbk_pos  = [float(sim_js_states.positions[idx]) for idx in self.idx_list]
        fbk_vel  = [float(sim_js_states.velocities[idx]) for idx in self.idx_list]

        response.joint_state.name = fbk_name
        response.joint_state.position = fbk_pos
        response.joint_state.velocity = fbk_vel
        print("反馈关节状态：", fbk_name, "\n", fbk_pos)
        return response
        
        
    def debug_callback(self, msg):
    # debug回调函数，可视化各种数据
        time.sleep(0.5)
        print("here")
        draw = _debug_draw.acquire_debug_draw_interface()
        draw.clear_points()
        # 绘制末端轨迹
        if len(msg.ee_pos_x) == 0:
            print("轨迹为空")
            return
        else:
            print("轨迹长度：", len(msg.ee_pos_x))
        point_list = list(zip(msg.ee_pos_x, msg.ee_pos_y, msg.ee_pos_z))
        colors = [(0,0,1,1) for _ in range(len(msg.ee_pos_x))]
        sizes = [10.0 for _ in range(len(msg.ee_pos_x))]
        draw.draw_points(point_list, colors, sizes)
        # 获取感知数据
        rgb_url = msg.rgb_url
        depth_url = msg.depth_url
        rgb_image = cv2.imread(rgb_url)
        depth_image = cv2.imread(depth_url, cv2.IMREAD_UNCHANGED)
        matrix = [0.0, 606.0, 246.0,
                -606.0, 0.0, 321.0,
                0.0, 0.0, 1.0]
        intrinsics = np.array([matrix]).reshape(3,3)
        # 相机内外参变换
        depth_data = np.array(depth_image, dtype=np.float32)
        h, w = depth_data.shape[:2]
        z = depth_data/1000
        # 构建像素坐标
        u = np.arange(w)
        v = np.arange(h)
        uu, vv = np.meshgrid(u, v)  # 都是 (H, W)
        # 展平成一维
        uu_flat = uu.ravel()
        vv_flat = vv.ravel()
        z_flat  = z.ravel()        # (H*W,)
        # 形成 3x(H*W) 的像素-深度矩阵
        pixels = np.vstack((uu_flat, vv_flat, np.ones_like(z_flat)))  # (3, H*W)
        # 通过内参矩阵反投影，再乘以深度
        K_inv = np.linalg.inv(intrinsics)
        xyz = (K_inv @ pixels) * z_flat
        # 转置
        cloud = xyz.T  # (H*W, 3)
        # 外参变换
        trans_matrix = np.array([
                                [0.57439011, -0.00879918, 0.81853441, -0.01584121],
                                [-0.01966464, -0.99980198, 0.00305147, 0.12688454],
                                [0.81834547, -0.01784892, -0.57444939, 1.30801121],
                                [0., 0., 0., 1.]
                                ])
        # 将点云重塑为(N, 3)
        points = cloud.reshape(-1, 3)
        # 添加齐次坐标维度 (N, 4)
        points_homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])
        # 应用变换矩阵 (注意：需要转置以匹配矩阵乘法规则)
        transformed_points_homogeneous = points_homogeneous @ trans_matrix.T
        # 转换回非齐次坐标 (除以w分量，虽然此处w=1)
        transformed_points = transformed_points_homogeneous[:, :3] / transformed_points_homogeneous[:, 3, np.newaxis]
        N = transformed_points.shape[0]
        point_list = []
        i = 0
        while i < N:
            if abs(transformed_points[i,0]) < 1 and abs(transformed_points[i,1]) < 0.5:
                point_list.append(transformed_points[i,:])
                i += 8
            else:
                i += 1
        # 绘制体素
        # point_list = list(zip(transformed_points[:,0], transformed_points[:,1], transformed_points[:,2]))
        #print(point_list)
        colors = [(1,1,1,1) for _ in range(N)]
        sizes = [20.0 for _ in range(N)]
        draw.draw_points(point_list, colors, sizes)
        


    def heartbeat_callback(self, msg):
        # 监听心跳消息
        self.heartbeat_received = True

    def check_heartbeat(self):
        # 检查心跳是否超时
        if not self.heartbeat_received:
            self.get_logger().info("没有收到心跳消息，重置仿真...")
            self.my_world.reset()
        # 重置心跳检测
        self.heartbeat_received = False

    def isaacsim_init(self):
    # 初始化isaac sim
        # 实例化仿真舞台，距离单位为米
        self.my_world = World(stage_units_in_meters=1.0)
        # 获取配置文件路径，位于功能包共享路径下
        package_name = 'motion_generator'
        package_share_directory = get_package_share_directory(package_name)
        robot_path = os.path.join(package_share_directory, 'data', 'konka.yml')
        body_path = os.path.join(package_share_directory, 'data', 'collision_body.yml')
        # 导入机器人yml配置文件，yml文件中包含urdf文件路径
        robot_cfg = load_yaml(robot_path)["robot_cfg"]
        self.joint_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
        self.default_config = robot_cfg["kinematics"]["cspace"]["retract_config"]
        # 将机器人添加到仿真舞台中
        self.robot,_ = add_robot_to_scene(robot_cfg, self.my_world, "/World/world_robot/")
        # 导入其他物体yml配置文件
        body_cfg = WorldConfig.from_dict(load_yaml(body_path))
        body_cfg_mesh = body_cfg.get_mesh_world()
        body_cfg_mesh.mesh[0].name += "_mesh"
        world_cfg = WorldConfig(cuboid=body_cfg_mesh.cuboid, mesh=body_cfg.mesh)
        # 将其他物体添加到仿真舞台中
        usd_help = UsdHelper()
        usd_help.load_stage(self.my_world.stage)
        usd_help.add_world_to_stage(world_cfg, base_frame="/World")
        # 添加地板
        self.my_world.scene.add_default_ground_plane()
        # 初始化仿真舞台(此步骤初始化多个对象)
        self.my_world.reset()
        # 关节id顺序
        self.idx_list = [self.robot.get_dof_index(x) for x in self.joint_names]
        print("关节id顺序:", self.idx_list)
        print("关节名称顺序:", self.joint_names)
        # 设置关节初始位置
        self.robot.set_joint_positions(self.default_config, self.idx_list)
        # 设置关节最大扭矩，保持仿真稳定
        self.robot._articulation_view.set_max_efforts(
            values=np.array([5000 for i in range(len(self.idx_list))]), joint_indices=self.idx_list)
        # 获取关节PD控制器
        self.robot._articulation_view.initialize()
        self.articulation_controller = self.robot.get_articulation_controller()
        #设置关节控制参数
        kps = np.array([60000., 60000., 60000., 60000., 60000., 2500.,
                60000., 2500., 2500., 2500., 60000., 60000.,
                60000., 60000., 60000.])
        kds = np.array([500., 500., 500., 500., 500., 1000., 500., 500.,
                500., 500., 28647.89, 28647.89, 28647.89, 500., 500.])
        self.articulation_controller.set_gains(kps, kds)
        # 电机控制信息
        self.cmd_motor_pos = np.zeros(len(self.idx_list))
        self.cmd_motor_vel = np.zeros(len(self.idx_list))

        

    def isaacsim_step(self):
    # 运行仿真平台
        self.my_world.step(render=True)
        # 如果仿真平台暂停，则跳过当前循环
        if not self.my_world.is_playing():
            self.my_world.reset()
            self.robot._articulation_view.initialize()
            self.cmd_motor_pos[:] = 0
            return
        # 获取跟踪的位置和姿态
        # cube_position, cube_orientation = self.target.get_world_pose()

        art_action = ArticulationAction(
            self.cmd_motor_pos,
            self.cmd_motor_vel,
            joint_indices=self.idx_list
        )
        #for _ in range(2):
        self.articulation_controller.apply_action(art_action)


def main():
    print(sys.version)
    rclpy.init()
    # executor = MultiThreadedExecutor(num_threads=2)
    simulatorNode = simulator()
    # executor.add_node(simulatorNode)
    try:
        # executor.spin()  # 执行器开始运行，处理ROS回调
        while rclpy.ok():
            simulatorNode.isaacsim_step()
            rclpy.spin_once(simulatorNode, timeout_sec=0.01)
    except KeyboardInterrupt:
        simulation_app.close()
        simulatorNode.get_logger().info("用户关闭节点...")
    finally:
        # executor.shutdown() 
        simulatorNode.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()




