#!/usr/bin/python

# CuRobo
# from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
# import sys
# sys.path.append('/home/konka/data/code/motion/src/motion_generator')

# import subprocess
# # 定义 source setup.bash 并执行其他命令
# bash_command = 'conda init && conda deactivate && source /home/ubuntu/XianglinZhang/develop/ros2_ws3/install/setup.bash && echo $ROS_DISTRO'
# # 使用 subprocess 执行 shell 命令
# process = subprocess.Popen(bash_command, shell=True, executable="/bin/bash", stdout=subprocess.PIPE, stderr=subprocess.PIPE)
# # 获取命令输出
# stdout, stderr = process.communicate()
# print("Output:", stdout.decode())
# print("Error:", stderr.decode())

# 此脚本为考虑深度图获取的障碍物的运动规划模块，目前还未在实机进行测试
import pdb

import curobo

print(curobo.__file__)

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig, Cuboid, Mesh, Capsule, Cylinder, Sphere
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
from visualization_msgs import Marker
from geometry_msgs.msg import Pose as PS

from motion_generator import MotionGenerator


class ObjectMotionGenerator(MotionGenerator):
    # todo:1.创建curbo的cuboid对象，保存为self.target_object 2.定义update_world函数，从request中解析出marker信息, 并更新self.target_object 调用motion_gen.update  3.将marker信息发布到ground_link下进行可视化
    def __init__(self):
        super().__init__()

        self.tensor_args = TensorDeviceType()

        world_cfg_table = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_table_on_jetson.yml"))
        )
        # print(join_path(get_world_configs_path(), "collision_table_on_jetson.yml"))
        world_cfg_table.cuboid[0].pose[0] += 0.02
        world_cfg_table.cuboid[0].pose[2] -= 0.0
        world_cfg1 = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_table_on_jetson.yml"))
        ).get_mesh_world()
        world_cfg1.mesh[0].name += "_mesh"
        world_cfg1.mesh[0].pose[2] = -10.5

        world_cfg = WorldConfig(cuboid=world_cfg_table.cuboid, mesh=world_cfg1.mesh)

        robot_cfg_path = get_robot_configs_path()
        self.robot_cfg = load_yaml(join_path(robot_cfg_path, "konka.yml"))["robot_cfg"]

        n_obstacle_cuboids = 30
        n_obstacle_mesh = None
        trajopt_dt = None
        optimize_dt = True
        trajopt_tsteps = 40
        trim_steps = None
        max_attempts = 4
        interpolation_dt = 0.08333

        motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.robot_cfg,
            world_cfg,
            self.tensor_args,
            collision_checker_type=CollisionCheckerType.MESH,
            num_trajopt_seeds=12,
            num_graph_seeds=12,
            interpolation_dt=interpolation_dt,
            collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
            optimize_dt=optimize_dt,
            trajopt_dt=trajopt_dt,
            trajopt_tsteps=trajopt_tsteps,
            trim_steps=trim_steps,
        )
        self.motion_gen = MotionGen(motion_gen_config)
        print("warming up...")
        self.motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False, parallel_finetune=True)

        print("Stable Motion Generator is Ready")

        self.plan_config = MotionGenPlanConfig(
            enable_graph=False,
            enable_graph_attempt=2,
            max_attempts=max_attempts,
            enable_finetune_trajopt=True,
            parallel_finetune=True,
            time_dilation_factor=0.3,
        )
        self.plan_id = 0
        self.initial_right_wheel_angular_position = 0.0
        self.initial_left_wheel_angular_position = 0.0

        self.target_object = Cuboid(
            name="target_object",
            pose=[0.0, 0.0, 10.0, 1.0, 0.0, 0.0, 0.0],  # x, y, z, qw, qx, qy, qz
            dims=[0.2, 1.0, 0.2],
            color=[0.8, 0.0, 0.0, 1.0],
        )

        self.update_joint_states()

    def update_target_object_collision(self, l_x, l_y, l_z, pos_x, pos_y, pos_z):
        self.target_object = [Cuboid(name="target_object", pose=[pos_x, pos_y, pos_z, 1, 0, 0, 0], dims=[l_x, l_y, l_z])]
        world = WorldConfig(cuboid=self.target_object)
        # self.motion_gen.world_coll_checker.clear_cache()
        self.motion_gen.update_world(world)
        print("目标物体位置及大小已更新")
        return

    def generate_plan_callback(self, request, response):
        current_time = self.get_clock().now().to_msg()
        self.update_joint_states()
        if self.joint_states is None:
            self.get_logger().warn('robot joint states are not published')

            response.is_success = False
            self.failure_reason = "robot joint states are not published"
            response.failure_reason = self.failure_reason
            return response

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
            print("request.target_pose", request.target_pose)
            target_position = target_pose.position
            target_orientation = target_pose.orientation

            # todo:提取marker.cube的信息并调用self.update_target_object_collision
            if request.marker.type == Marker.Cube:
                print("解析marker并更新世界")
                self.update_target_object_collision(
                    l_x=request.marker.scale.x,
                    l_y=request.marker.scale.y,
                    l_z=request.marker.scale.z,
                    pos_x=request.marker.pose.position.x,
                    pos_y=request.marker.pose.position.y,
                    pos_z=request.marker.pose.position.z
                )
            else:
                print("该模块只支持Cube消息类型来更新物体的碰撞,轨迹将忽略目标物体的碰撞")
                pass

            self.ee_translation_goal = np.array([
                target_position.x,
                target_position.y,
                target_position.z  # 0.09为规划补偿
            ])

            self.ee_orientation_teleop_goal = np.array([
                target_orientation.w,
                target_orientation.x,
                target_orientation.y,
                target_orientation.z
            ])

            # 测试采用不同方法生成目标位姿
            goal_pose = Pose.from_list(
                [target_position.x, target_position.y, target_position.z + 0.0, target_orientation.w,
                 target_orientation.x, target_orientation.y, target_orientation.z])

            # # 生成逆运动学目标
            # ik_goal = Pose(
            #     position=self.tensor_args.to_device(self.ee_translation_goal),
            #     quaternion=self.tensor_args.to_device(self.ee_orientation_teleop_goal),
            # )

            # 实例化cumotion需要的JointState
            # cu_js = JointState(
            #     position=self.tensor_args.to_device(self.joint_positions),
            #     velocity=self.tensor_args.to_device(self.joint_velocities),
            #     acceleration=self.tensor_args.to_device(self.joint_velocities) * 0.0,
            #     jerk=self.tensor_args.to_device(self.joint_velocities) * 0.0,
            #     joint_names=self.joint_names,
            # )

            cu_js = JointState.from_position(
                joint_names=self.joint_names,
                position=self.tensor_args.to_device(self.joint_positions)
            )

            cu_js.velocity *= 0.0
            cu_js.acceleration *= 0.0

            # print(self.motion_gen.kinematics.joint_names)
            # pdb.set_trace()
            # cu_js = cu_js.get_ordered_joint_state(self.motion_gen.kinematics.joint_names)
            self.plan_config.pose_cost_metric = self.pose_metric

            # curobo计算无碰撞的轨迹
            # result = self.motion_gen.plan_single(cu_js, ik_goal, self.plan_config)
            result = self.motion_gen.plan_single(cu_js, goal_pose, self.plan_config)
            succ = result.success.item()
            self.is_generation_success = succ

            if succ:
                print("motion_gen.result.position_error = ", result.position_error)
                self.plan_id += 1
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
        self.future = self.client.call_async(self.joint_states_req)
        self.future.add_done_callback(self.joint_states_callback)

    def joint_states_callback(self, future):
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
            if joint_name in cspace_joint_names:
                index = joint_names.index(joint_name)
                filtered_joint_names.append(joint_name)
                filtered_joint_positions.append(joint_positions[index])
                filtered_joint_velocities.append(joint_velocities[index])

            if joint_name == "joint_arm_L0":

                filtered_joint_names.append("joint_arm_l2")
                filtered_joint_names.append("joint_arm_l1")
                filtered_joint_names.append("joint_arm_l0")
                index = joint_names.index(joint_name)
                for i in range(3):
                    filtered_joint_positions.append(joint_positions[index] / 3.0)
                    filtered_joint_velocities.append(joint_velocities[index])

            # get qpos of left_wheel and right_wheel and use them to calculate base joint position
            if joint_name == "joint_right_wheel" and self.plan_id == 0:
                index = joint_names.index(joint_name)
                self.initial_right_wheel_angular_position = joint_positions[index]
            if joint_name == "joint_left_wheel" and self.plan_id == 0:
                index = joint_names.index(joint_name)
                self.initial_left_wheel_angular_position = joint_positions[index]
        base_z_position = self.calculate_base_z_position(joint_names, joint_positions)

        filtered_joint_names.insert(0, "base_z")
        filtered_joint_positions.insert(0, base_z_position)

        # filtered_joint_positions[filtered_joint_names.index("joint_roll")] = 0.0
        # filtered_joint_positions[filtered_joint_names.index("joint_pitch")] = 0.0
        # filtered_joint_positions[filtered_joint_names.index("joint_yaw")] = 0.0

        self.joint_names = filtered_joint_names
        # self.joint_positions = np.array(filtered_joint_positions, dtype=np.float32)
        # self.joint_velocities = np.array(filtered_joint_velocities, dtype=np.float32)

        self.joint_positions = [filtered_joint_positions]
        self.joint_velocities = [filtered_joint_velocities]

        print("[debug]", self.joint_names)
        print("[debug]", self.joint_positions)

        self.last_joint_state_update_time = self.get_clock().now().to_msg()

    def calculate_base_z_position(self, joint_names, joint_positions):
        '''
        :param List:joint_names
        :param List:joint_positions
        :return: base_z joint_state
        '''
        right_wheel_angular_position = joint_positions[joint_names.index(
            'joint_right_wheel')] - self.initial_right_wheel_angular_position
        left_wheel_angular_position = joint_positions[joint_names.index(
            'joint_left_wheel')] - self.initial_left_wheel_angular_position
        base_z_position = (
                                      right_wheel_angular_position * self.wheel_radius - left_wheel_angular_position * self.wheel_radius) / self.track_width
        return base_z_position


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMotionGenerator()
    logger = rclpy.logging.get_logger('logger')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info('interrupt received, so shutting down')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()