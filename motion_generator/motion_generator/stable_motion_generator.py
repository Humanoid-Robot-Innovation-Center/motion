#!/usr/bin/python

# CuRobo
# from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
import sys
import os
from ament_index_python.packages import get_package_share_directory

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.types.state import JointState

from curobo.util_file import (
    get_assets_path,
    get_filename,
    get_path_of_dir,
    get_robot_configs_path,
    get_world_configs_path,
    join_path,
    load_yaml,
)
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
    PoseCostMetric,
)
# third party
import numpy as np
import time
# ROS2
import rclpy
from rclpy.node import Node

from gongga_interface.srv import GeneratePlan
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import JointState as JS
from geometry_msgs.msg import Pose as PS

from motion_generator.motion_generator import MotionGenerator


class StableMotionGenerator(MotionGenerator):
    def __init__(self):
        super().__init__()
        #hardware device type
        self.tensor_args = TensorDeviceType()
        # 新的路径
        package_name = 'motion_generator'
        package_share_directory = get_package_share_directory(package_name)
        #获取世界配置
        body_path = os.path.join(package_share_directory, 'data', 'collision_body.yml')
        body_cfg = WorldConfig.from_dict(load_yaml(body_path))
        body_cfg_mesh = body_cfg.get_mesh_world()
        body_cfg_mesh.mesh[0].name += "_mesh"
        world_cfg = WorldConfig(cuboid=body_cfg_mesh.cuboid, mesh=body_cfg.mesh)
        
        # 初始化motion_gen模块
        n_obstacle_cuboids = 30
        n_obstacle_mesh = 100
        trajopt_dt = None  # default:None
        optimize_dt = True
        trajopt_tsteps = 32  # default:32
        trim_steps = None
        max_attempts = 8
        interpolation_dt = 0.08333
        #获取机器人配置
        robot_path = os.path.join(package_share_directory, 'data', 'konka.yml')
        self.robot_cfg = load_yaml(robot_path)["robot_cfg"]
        
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.robot_cfg,
            world_cfg,
            self.tensor_args,
            collision_checker_type=CollisionCheckerType.MESH,
            num_trajopt_seeds=12,
            num_graph_seeds=12,
            interpolation_dt=interpolation_dt,
            collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
            maximum_trajectory_dt=1.5,
            optimize_dt=optimize_dt,
            trajopt_dt=trajopt_dt,
            trajopt_tsteps=trajopt_tsteps,
            trim_steps=trim_steps,
        )
        self.motion_gen = MotionGen(motion_gen_config)
        print("stable motion generator warming up...")
        self.motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False, parallel_finetune=True)
        print("stable motion generator is Ready")

        #求解器配置
        self.plan_config = MotionGenPlanConfig(
            enable_graph=False,
            enable_graph_attempt=2,
            max_attempts=max_attempts,
            enable_finetune_trajopt=True,
            parallel_finetune=True,
            time_dilation_factor=0.5,
        )

    def generate_plan_callback(self, request, response):
        
        if request.task == "get_joint_states":
            self.update_joint_states()
            return response

        if self.joint_states is None:
            response.is_success = False
            self.failure_reason = "robot joint states are not published"
            response.failure_reason = self.failure_reason
            return response

        # elif (current_time.sec + current_time.nanosec * 1e-9) - (
        #         self.last_joint_state_update_time.sec + self.last_joint_state_update_time.nanosec * 1e-9) > 1.0:
        #     time_diff = (current_time.sec + current_time.nanosec * 1e-9) - (
        #             self.last_joint_state_update_time.sec + self.last_joint_state_update_time.nanosec * 1e-9)
        #     self.get_logger().warn(
        #         f'Joint state is too old. Time difference: {time_diff:.2f} seconds')
        #
        #     response.is_success = False
        #     self.failure_reason = "robot joint states is too old"
        #     response.failure_reason = self.failure_reason
        #     return response

        elif (np.max(np.abs(self.joint_velocities)) > 0.2):
            self.get_logger().warn('robot have to be static before motion generation')

            response.is_success = False
            self.failure_reason = "robot is still moving"
            response.failure_reason = self.failure_reason
            return response
        else:
            # 提取 position 和 orientation
            # target_pose -> geometry_msgs.msg.Pose
            target_pose = request.target_pose
            target_position = target_pose.position
            target_orientation = target_pose.orientation
            print("[debug]:目标点:", target_position)
            print("[debug]:关节名：", self.joint_names)
            print("[debug]:当前关节位置", self.joint_positions)
            #几何坐标
            self.ee_translation_goal = np.array([
                target_position.x,
                target_position.y,
                target_position.z
            ], dtype=np.float32)
            #四元数
            self.ee_orientation_teleop_goal = np.array([
                target_orientation.w,
                target_orientation.x,
                target_orientation.y,
                target_orientation.z
            ], dtype=np.float32)

            # 生成逆运动学目标
            ik_goal = Pose(
                position=self.tensor_args.to_device(self.ee_translation_goal),
                quaternion=self.tensor_args.to_device(self.ee_orientation_teleop_goal),
            )

            # 实例化cumotion需要的JointState
            cu_js = JointState(
                position=self.tensor_args.to_device(self.joint_positions),
                velocity=self.tensor_args.to_device(self.joint_velocities) * 0.0,
                acceleration=self.tensor_args.to_device(self.joint_velocities) * 0.0,
                jerk=self.tensor_args.to_device(self.joint_velocities) * 0.0,
                joint_names=self.joint_names,
            )

            #父类初始化中设为None
            self.plan_config.pose_cost_metric = self.pose_metric

            # curobo计算无碰撞的轨迹
            result = self.motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self.plan_config)
            succ = result.success.item()
            self.is_generation_success = succ

            if succ:
                self.cmd_plan = result.get_interpolated_plan()
                self.cmd_plan = self.motion_gen.get_full_js(self.cmd_plan)
                joint_trajectory = self.get_response_joint_trajectory()

                response.is_success = True
                response.joint_trajectory = joint_trajectory

                return response
            else:
                self.get_logger().warn("plan未收敛到解: " + str(result.status))
                self.failure_reason = str(result.status)
                response.is_success = False
                response.failure_reason = self.failure_reason
                return response

    def attach_object_to_kinematics(self):
        # todo:not implement yet
        pass

    def detach_object_from_kinematics(self):
        # todo:not implement yet
        pass

    def update_joint_states(self):
        # 获取关节当前状态
        future = self.client.call_async(self.joint_states_req)
        future.add_done_callback(self.update_joint_callback)
    
    def update_joint_callback(self, future):
        try:
            response = future.result()
            joint_states_response = response.joint_state
        except Exception as e:
            self.get_logger().error(f"Joint States Service call failed: {e}")

        self.joint_states = joint_states_response
        joint_names = joint_states_response.name

        # 保存关节位置和速度为 NumPy 数组
        joint_positions = np.array(joint_states_response.position)
        joint_velocities = np.array(joint_states_response.velocity)

        cspace_joint_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]

        # 初始化新的joint_positions和joint_velocities
        filtered_joint_names = []
        filtered_joint_positions = []
        filtered_joint_velocities = []

        # 遍历cspace_joint_names, 按顺序筛选并存储
        for joint_name in joint_names:
            index = joint_names.index(joint_name)
            if joint_name == "joint_arm_L0":
                filtered_joint_names.append("joint_arm_l2")
                filtered_joint_names.append("joint_arm_l1")
                filtered_joint_names.append("joint_arm_l0")
                for i in range(3):
                    filtered_joint_positions.append(joint_positions[index] / 3.0)
                    filtered_joint_velocities.append(joint_velocities[index])
            # 感知数据是基于当前机器人坐标系的，此时base_z的值应当为0
            # 因此，调用规划器时，应当更新感知数据，否则会出错
            # 如果是仿真，在运动完毕后应该重置仿真舞台
            elif joint_name == "base_z":
                filtered_joint_names.append("base_z")
                filtered_joint_positions.append(0)
                filtered_joint_velocities.append(joint_velocities[index])
            else:
                if joint_name in cspace_joint_names:
                    filtered_joint_names.append(joint_name)
                    filtered_joint_positions.append(joint_positions[index])
                    filtered_joint_velocities.append(joint_velocities[index])

        self.joint_names = filtered_joint_names
        self.joint_positions = np.array(filtered_joint_positions)
        self.joint_velocities = np.array(filtered_joint_velocities)
    
        print("反馈关节名：", self.joint_names)
        print("反馈关节位置：", self.joint_positions)


def main(args=None):
    rclpy.init(args=args)
    node = StableMotionGenerator()
    logger = rclpy.logging.get_logger('logger')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户关闭节点...')
    finally:
        node.destroy_node()



if __name__ == "__main__":
    main()
