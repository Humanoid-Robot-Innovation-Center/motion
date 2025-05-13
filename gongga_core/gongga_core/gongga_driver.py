# ROS2
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from motor_interface.msg import VelAndPose  # 自定义消息类型，用于控制电机
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
        self.base_z_pos_publisher = self.create_publisher(VelAndPose, "base_z_pos", 10)
        self.base_z_vel_publisher = self.create_publisher(VelAndPose, "cmd_vel_pose", 10) # todo:修改话题为二代机底盘位置控制的话题
        self.wrist_yaw_publisher = self.create_publisher(VelAndPose, "wrist_yaw", 10)
        self.wrist_pitch_publisher = self.create_publisher(VelAndPose, "wrist_pitch", 10)
        self.wrist_roll_publisher = self.create_publisher(VelAndPose, "wrist_roll", 10)
        self.arm_publisher = self.create_publisher(VelAndPose, "cmd_arm", 10)
        self.lift_publisher = self.create_publisher(VelAndPose, "cmd_lift", 10)
        self.gripper_publisher = self.create_publisher(VelAndPose, "gripper_pull", 10)

        self.main_group = ReentrantCallbackGroup()

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
            node.get_logger().info('用户关闭节点...')
        finally:
            node.destroy_node()  # 清理节点
            executor.shutdown()  # 无论是否捕获异常，都关闭执行器
    except Exception as e:
        print(f"Error occurred: {e}")



if __name__ == "__main__":
    main()



