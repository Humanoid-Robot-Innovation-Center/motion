# ROS2
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from motor_interface.msg import VelAndPose  # 自定义消息类型，用于控制电机
# from gongga_interface.srv import GripperControl
import numpy as np
from gongga_core.helpers.diver_helper import euler_to_quaternion, create_default_motor_msg, convert_trajectory_message
from gongga_core.joint_trajectory_server import JointTrajectoryAction

from sensor_msgs.msg import JointState as JS
from typing import Dict
import numpy as np


class GonggaDriver(Node):
    def __init__(self):
        super().__init__('gongga_driver')
        # todo:assign calibration offset 以及根据offset进行补偿的逻辑
        self.is_open_loop_control = True
        self.trajectory_mode = True

        # todo:根据第二代机器人修改这些关节限制参数
        self.lift_limit = 1.04
        self.arm_limit = 0.48

        self.joint_trajectory = {
            "position_trajectory": None,
            "velocity_trajectory": None
        }

        self.ros_setup()

    def ros_setup(self):
        # create topic subscribed by motor driver node
        self.base_z_vel_publisher = self.create_publisher(VelAndPose, "cmd_vel_pose", 10)
        self.base_z_pos_publisher = self.create_publisher(VelAndPose, "cmd_vel_pose", 10) # todo:修改话题为二代机底盘位置控制的话题
        self.wrist_yaw_publisher = self.create_publisher(VelAndPose, "wrist_yaw", 10)
        self.wrist_pitch_publisher = self.create_publisher(VelAndPose, "wrist_pitch", 10)
        self.wrist_roll_publisher = self.create_publisher(VelAndPose, "wrist_roll", 10)
        self.arm_publisher = self.create_publisher(VelAndPose, "cmd_arm", 10)
        self.lift_publisher = self.create_publisher(VelAndPose, "cmd_lift", 10)
        self.gripper_publisher = self.create_publisher(VelAndPose, "gripper_pull", 10)

        # self.srv = self.create_service(GripperControl, 'gripper_control', self.gripper_state_callback)

        self.main_group = ReentrantCallbackGroup()

    #     if self.trajectory_mode:
    #         self.trajectory_srv = self.create_service(ExecuteTrajectory, 'ExecuteTrajectory', self.trajectory_callback)

    # def trajectory_callback(self, request, response):
    
    #     joint_trajectory = convert_trajectory_message(request.trajectory)
    
    #     is_beyond_limits = self.check_beyond_limits(joint_trajectory)
    
    #     if is_beyond_limits:
    #         response.is_success = False
    #         response.failure_reason = "goal position out of joint limits "
    #     else:
    #         response.is_success = True
    #         if self.is_open_loop_control:
    #             self.execute_trajectory_open_loop(request.control_rate, joint_trajectory)
    #         else:
    #             self.execute_trajectory_close_loop(joint_trajectory)
    #     return response

    # def execute_trajectory_open_loop(self, rate, joint_trajectory):
    #     control_time = 1.0/rate
    #     cmd_idx = 0
    #     velocity_trajectory = joint_trajectory["velocity_trajectory"]
    #     joint_position_trajectory = joint_trajectory["position_trajectory"]
    #
    #     while len(velocity_trajectory) != 0 and cmd_idx < len(velocity_trajectory):
    #         start_time = time.time()
    #         joint_velocities = velocity_trajectory[cmd_idx]
    #         joint_positions = joint_position_trajectory[cmd_idx]
    #         # 将joint_positions和joint_velocities按一定控制周期发布
    #         arm_vel = 0.0
    #         for i, name in enumerate(joint_trajectory["joint_names"]):
    #             if name == 'base_z':
    #                 base_z_action = create_default_motor_msg()
    #                 base_z_action.mode = 0
    #                 base_z_action.vel.angular.z = joint_velocities[i]
    #                 self.base_z_vel_publisher.publish(base_z_action)
    #                 # print("base_z_vel", joint_velocities[i])
    #                 # print("step:", self.cmd_idx)
    #                 print("发布base_z命令", base_z_action.vel.angular.z)
    #             elif name == 'jointshou':
    #                 # # 位置控制
    #                 # joint_lift_position = joint_positions[i]
    #                 # last_joint_lift_position = self.last_joint_positions[i]
    #                 # joint_lift_action = self.create_default_motor_msg()
    #                 # joint_lift_action.pose.position.z = joint_lift_position - last_joint_lift_position
    #                 # self.lift_publisher.publish(joint_lift_action)
    #                 # 速度控制
    #                 joint_lift_vel = joint_velocities[i]
    #                 joint_lift_action = create_default_motor_msg()
    #                 joint_lift_action.mode = 0
    #                 joint_lift_action.vel.linear.z = joint_lift_vel
    #                 self.lift_publisher.publish(joint_lift_action)
    #                 print("发布抬升命令", joint_lift_action.vel.linear.z)
    #                 print("!!!!!!joint_lift_vel:", joint_lift_vel)
    #             elif name == 'jointss1':
    #                 arm_vel += joint_velocities[i]
    #             elif name == 'jointss2':
    #                 arm_vel += joint_velocities[i]
    #             elif name == 'jointss3':
    #                 arm_vel += joint_velocities[i]
    #             elif name == 'jointsw1':
    #                 sw1_position = joint_positions[i]
    #                 sw1_action = create_default_motor_msg()
    #                 sw1_action.pose.orientation.w, sw1_action.pose.orientation.x, sw1_action.pose.orientation.y, sw1_action.pose.orientation.z = euler_to_quaternion(
    #                     0, 0, sw1_position)
    #                 self.wrist_yaw_publisher.publish(sw1_action)
    #             elif name == 'jointsw2':
    #                 sw2_position = joint_positions[i]
    #                 sw2_action = create_default_motor_msg()
    #                 sw2_action.pose.orientation.w, sw2_action.pose.orientation.x, sw2_action.pose.orientation.y, sw2_action.pose.orientation.z = euler_to_quaternion(
    #                     0, 0, sw2_position)
    #                 self.wrist_pitch_publisher.publish(sw2_action)
    #             elif name == 'jointsw4':
    #                 sw4_position = joint_positions[i]
    #                 sw4_action = create_default_motor_msg()
    #                 sw4_action.pose.orientation.w, sw4_action.pose.orientation.x, sw4_action.pose.orientation.y, sw4_action.pose.orientation.z = euler_to_quaternion(
    #                     0, 0, sw4_position)
    #                 self.wrist_roll_publisher.publish(sw4_action)
    #
    #         arm_action = create_default_motor_msg()
    #         arm_action.mode = 0
    #         arm_action.vel.linear.x = arm_vel
    #
    #         self.arm_publisher.publish(arm_action)
    #         cmd_idx += 1
    #         time.sleep(control_time)
    #         end_time = time.time()
    #         total_time = end_time - start_time
    #         print("用时：", total_time)
    #         print("cmd_idx:", cmd_idx)

    def execute_trajectory_close_loop(self, joint_trajectory):
        # todo:not implement yet
        print("close_loop_control not implement yet, please modify self.is_open_loop_control as True")
        pass

    def check_beyond_limits(self, joint_trajectory):
        # todo:check velocity limits
        is_beyond_limits = False

        lift_joint_index = joint_trajectory["joint_names"].index('joint_lift')
        arm_joint1_index = joint_trajectory["joint_names"].index('joint_arm_l2')
        arm_joint2_index = joint_trajectory["joint_names"].index('joint_arm_l1')
        arm_joint3_index = joint_trajectory["joint_names"].index('joint_arm_l0')

        if joint_trajectory["position_trajectory"][-1][lift_joint_index] >= self.lift_limit:
            is_beyond_limits = True
            print("beyond lift joint limit")
        elif joint_trajectory["position_trajectory"][-1][arm_joint1_index]+joint_trajectory["position_trajectory"][-1][arm_joint2_index]+joint_trajectory["position_trajectory"][-1][arm_joint3_index] >= self.arm_limit:
            is_beyond_limits = True
            print("beyond arm joint limit")

        return is_beyond_limits

    def gripper_state_callback(self, request, response):
        gripper_state = request.data
        if gripper_state == 0:
            gripper_action = VelAndPose()
            gripper_action.mode = 1
            gripper_action.pose.orientation.x = 0.0
            gripper_action.pose.orientation.y = 0.0
            gripper_action.pose.orientation.z = -0.707
            gripper_action.pose.orientation.w = 0.707
            self.get_logger().info('闭合夹爪')
            response.message = 'Gripper closed'
        elif gripper_state == 1:
            gripper_action = VelAndPose()
            gripper_action.mode = 1
            gripper_action.pose.orientation.x = 0.0
            gripper_action.pose.orientation.y = 0.0
            gripper_action.pose.orientation.z = 0.954
            gripper_action.pose.orientation.w = 0.301
            self.get_logger().info('张开夹爪')
            response.is_closed = False

        self.gripper_publisher.publish(gripper_action)
        return response


def main():
    try:
        rclpy.init()
        executor = MultiThreadedExecutor(num_threads=5)
        node = GonggaDriver()
        node.joint_trajectory_action = JointTrajectoryAction(node)
        executor.add_node(node)

        try:
            executor.spin()  # 执行器开始运行，处理ROS回调
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard Interrupt, shutting down node...")
        finally:
            executor.shutdown()  # 无论是否捕获异常，都关闭执行器
            node.destroy_node()  # 清理节点
            rclpy.shutdown()  # 关闭rclpy
    except Exception as e:
        print(f"Error occurred: {e}")



if __name__ == "__main__":
    main()



