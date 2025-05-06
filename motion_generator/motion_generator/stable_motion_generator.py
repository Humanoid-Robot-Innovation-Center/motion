#!/usr/bin/python

# CuRobo
# from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
import sys
sys.path.append('/home/konka/data/code/motion/curobo/build/lib.linux-aarch64-3.10/curobo/curobolib')
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

        self.tensor_args = TensorDeviceType()

        world_cfg_table = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
        )
        world_cfg_table.cuboid[0].pose[2] -= 10.0
        world_cfg1 = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
        ).get_mesh_world()
        world_cfg1.mesh[0].name += "_mesh"
        world_cfg1.mesh[0].pose[2] = -10.5
        world_cfg = WorldConfig(cuboid=world_cfg_table.cuboid, mesh=world_cfg1.mesh)

        # 初始化motion_gen模块
        n_obstacle_cuboids = 30
        n_obstacle_mesh = None
        trajopt_dt = None  # default:None
        optimize_dt = True
        trajopt_tsteps = 32  # default:32
        trim_steps = None
        max_attempts = 8
        interpolation_dt = 0.08333
        self.dt = interpolation_dt
        robot_cfg_path = get_robot_configs_path()
        self.robot_cfg = load_yaml(join_path(robot_cfg_path, "konka.yml"))["robot_cfg"]
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
        print("stable motion generator warming up...")
        self.motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False, parallel_finetune=True)
        print("stable motion generator is Ready")

        self.plan_config = MotionGenPlanConfig(
            enable_graph=False,
            enable_graph_attempt=2,
            ik_seeds = 64,
            max_attempts=max_attempts,
            enable_finetune_trajopt=True,
            parallel_finetune=True,
            time_dilation_factor=0.5,
        )

        self.plan_id = 0
        self.initial_right_wheel_angular_position = 0.0
        self.initial_left_wheel_angular_position = 0.0

        self.update_joint_states()

    def generate_plan_callback(self, request, response):
        current_time = self.get_clock().now().to_msg()
        self.update_joint_states()
        if self.joint_states is None:
            self.get_logger().warn('robot joint states are not published')

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

            self.ee_translation_goal = np.array([
                target_position.x,
                target_position.y,
                target_position.z
            ], dtype=np.float32)

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

            self.plan_config.pose_cost_metric = self.pose_metric

            # curobo计算无碰撞的轨迹
            result = self.motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self.plan_config)
            succ = result.success.item()
            self.is_generation_success = succ

            if succ:
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

        self.joint_names = filtered_joint_names
        self.joint_positions = np.array(filtered_joint_positions)

        self.joint_velocities = np.array(filtered_joint_velocities)

        print("[debug]", base_z_position)
        print("[debug]", self.joint_names)
        print("[debug]", self.joint_positions)

        self.last_joint_state_update_time = self.get_clock().now().to_msg()

    def calculate_base_z_position(self, joint_names, joint_positions):
        '''
        :param List:joint_names
        :param List:joint_positions
        :return: base_z joint_state
        '''
        right_wheel_angular_position = joint_positions[joint_names.index('joint_right_wheel')] - self.initial_right_wheel_angular_position
        left_wheel_angular_position = joint_positions[joint_names.index('joint_left_wheel')] - self.initial_left_wheel_angular_position
        base_z_position = (right_wheel_angular_position * self.wheel_radius - left_wheel_angular_position * self.wheel_radius) / self.track_width
        return base_z_position


def main(args=None):
    rclpy.init(args=args)
    node = StableMotionGenerator()
    logger = rclpy.logging.get_logger('logger')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info('interrupt received, so shutting down')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
