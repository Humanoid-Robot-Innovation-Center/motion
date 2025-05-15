import time
import threading

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from sensor_msgs.msg import JointState  # Assuming you are using JointState for trajectory
from gongga_core.helpers.diver_helper import euler_to_quaternion, create_default_motor_msg, convert_trajectory_message

from gongga_interface.action import FollowJointTrajectory
import numpy as np


class JointTrajectoryAction:
    def __init__(self, node):
        self.node = node
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.latest_goal_id = 0  # 用于跟踪目标ID

        # 创建 Action Server
        self.server = ActionServer(
            self.node,
            FollowJointTrajectory,
            '/gongga_controller/follow_joint_trajectory',
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
            goal_callback=self.goal_cb,
            handle_accepted_callback=self.handle_accepted_cb,
            callback_group=node.main_group
        )

    # 检查目标的回调
    def goal_cb(self, goal_handle):

        goal = goal_handle
        print("收到目标")
        joint_names = goal.trajectory.name
        #print("goal.trajectory:", goal.trajectory)
        # joint_trajectory = convert_trajectory_message(goal.trajectory)
        #print("trajectory converted")

        # is_beyond_limits = self.check_beyond_limits(joint_trajectory)

        buffer_time = 0.5  # 缓冲时间，单位秒
        self.node.get_logger().info(f"等待 {buffer_time} 秒以接收完整的目标数据。")
        # 使用 threading.Timer 延迟调用 goal_handle.execute()以确保轨迹接收完成
        time.sleep(buffer_time)

        is_beyond_limits = False
        print("trajectory safe")

        if is_beyond_limits:

            result = FollowJointTrajectory.Result()
            result.is_success = False
            self.node.get_logger().info("Rejected goal trajectory with some position out of joint limits.")
            result.failure_reason = "trajectory out of joint limits"

            goal_handle.abort(result)  # 使用带有自定义消息的结果拒绝目标

            return GoalResponse.REJECT
        else:
            self.node.get_logger().info("Accepted goal trajectory.")
            return GoalResponse.ACCEPT

    # 当目标被接受时的回调
    def handle_accepted_cb(self, goal_handle):
        with self._goal_lock:
            # 如果有已经活动的目标，则终止旧目标
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.node.get_logger().info('Aborting previous goal')
                # self._goal_handle.abort()  # 如果存在状态转换问题，可省略这行
            self._goal_handle = goal_handle

        self.latest_goal_id += 1  # 更新最新的 goal_id

        # 开启一个线程来执行目标
        goal_handle.execute()

    # 执行轨迹的回调
    def execute_cb(self, goal_handle):
        self.node.get_logger().info("Executing goal trajectory...")

        goal_id = self.latest_goal_id

        if self.node.is_open_loop_control:
            self.execute_trajectory_open_loop(goal_handle, goal_id)
        else:
            self.execute_trajectory_close_loop(goal_handle)

        # 目标完成，返回成功结果
        result = FollowJointTrajectory.Result()
        self.node.get_logger().info("Goal successfully completed.")
        self.node.get_logger().info("-------------------------------------------")
        self.node.get_logger().info("Finished goal")
        result.is_success = True
        goal_handle.succeed()
        return result

    # 取消的回调
    def cancel_cb(self, goal_handle):
        self.node.get_logger().info("Received cancel request.")
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.stop_trajectory()
                return CancelResponse.ACCEPT
            else:
                return CancelResponse.REJECT

    def success_callback(self, goal_handle, success_str):
        print("-------------------------------------------")
        print("Finished goal")
        result = FollowJointTrajectory.Result()
        result.is_success = True
        result.failure_reason = success_str
        goal_handle.succeed()
        return result

    def check_beyond_limits(self, joint_trajectory):
        # todo:check velocity limits
        is_beyond_limits = False

        lift_joint_index = joint_trajectory["joint_names"].index('joint_lift')
        arm_joint1_index = joint_trajectory["joint_names"].index('joint_arm_l2')
        arm_joint2_index = joint_trajectory["joint_names"].index('joint_arm_l1')
        arm_joint3_index = joint_trajectory["joint_names"].index('joint_arm_l0')

        for i in range(len(joint_trajectory["position_trajectory"])):
            if joint_trajectory["position_trajectory"][i][lift_joint_index] >= self.node.lift_limit:
                is_beyond_limits = True
            elif joint_trajectory["position_trajectory"][i][arm_joint1_index] + \
                    joint_trajectory["position_trajectory"][i][arm_joint2_index] + \
                    joint_trajectory["position_trajectory"][i][arm_joint3_index] >= self.node.arm_limit:
                is_beyond_limits = True

        return is_beyond_limits

    def calculate_feedforward_term(self, joint_trajectory):
        lift_joint_index = joint_trajectory["joint_names"].index('joint_lift')
        arm_joint1_index = joint_trajectory["joint_names"].index('joint_arm_l2')
        arm_joint2_index = joint_trajectory["joint_names"].index('joint_arm_l1')
        arm_joint3_index = joint_trajectory["joint_names"].index('joint_arm_l0')

        if joint_trajectory["position_trajectory"][-1][arm_joint1_index] + joint_trajectory["position_trajectory"][-1][arm_joint2_index] + joint_trajectory["position_trajectory"][-1][arm_joint3_index] >= 0.2:
            for i in range(len(joint_trajectory["position_trajectory"])):
                joint_trajectory["position_trajectory"][i][lift_joint_index] += (0.03 / len(
                    joint_trajectory["position_trajectory"]))*i
            print("已补偿抬升量")
        return joint_trajectory

    def execute_trajectory_open_loop(self, goal_handle, goal_id):
        joint_trajectory = convert_trajectory_message(goal_handle.request.trajectory)
        # 补偿由伸缩臂伸长引起的夹爪下垂
        # joint_trajectory = self.calculate_feedforward_term(joint_trajectory)

        control_rate = goal_handle.request.control_rate

        control_time = 1.0/control_rate
        cmd_idx = 0
        velocity_trajectory = joint_trajectory["velocity_trajectory"]
        joint_position_trajectory = joint_trajectory["position_trajectory"]

        if velocity_trajectory is None:
            self.stop_trajectory()
            return FollowJointTrajectory.Result()
        
        self.base_z_prev = 0.0
        
        while len(velocity_trajectory) != 0 and cmd_idx < len(velocity_trajectory):

            if goal_id != self.latest_goal_id:
                self.node.get_logger().info(
                    "{0} joint_traj action: PREEMPTION REQUESTED, but not stopping current motions to allow smooth interpolation between old and new commands.")
                # self.node.stop_the_robot = False
                # self.node.robot_mode_rwlock.release_read()
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            if not goal_handle.is_active:
                self.node.get_logger().info('Goal aborted')
                self.stop_trajectory()
                return FollowJointTrajectory.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_trajectory()
                self.node.get_logger().info(f"Goal canceled at point {cmd_idx}.")
                return FollowJointTrajectory.Result()

            start_time = time.time()
            joint_velocities = velocity_trajectory[cmd_idx]
            joint_positions = joint_position_trajectory[cmd_idx]

            # publish feedback (progress)
            feedback = FollowJointTrajectory.Feedback()
            feedback.progress_percentage = cmd_idx/len(velocity_trajectory)
            # self.node.get_logger().info(f"Executing point {cmd_idx}.")
            goal_handle.publish_feedback(feedback)

            # 将joint_positions和joint_velocities按一定控制周期发布
            arm_vel = 0.0
            arm_pos = 0.0
            for i, name in enumerate(joint_trajectory["joint_names"]):
                if name == 'base_z':
                    base_z_pos = joint_positions[i]
                    rel_z_move = base_z_pos - self.base_z_prev
                    self.base_z_prev = base_z_pos
                    base_z_action = create_default_motor_msg()
                    base_z_action.mode = 1
                    w,x,y,z = euler_to_quaternion(0, 0, rel_z_move)
                    base_z_action.pose.orientation.w = float(w)
                    base_z_action.pose.orientation.x = float(x)
                    base_z_action.pose.orientation.y = float(y)
                    base_z_action.pose.orientation.z = float(z)
                    # base_z_action.pose.orientation.w,
                    # base_z_action.pose.orientation.x,
                    # base_z_action.pose.orientation.y,
                    # base_z_action.pose.orientation.z = euler_to_quaternion(0, 0, base_z_pos)
                    self.node.base_z_pos_publisher.publish(base_z_action)
                    # print("[debug]base_z_orient_z:", base_z_action.pose.orientation.z)
                    # print("[debug]base_z:", base_z_pos)
                    print("[debug]:yaw  = ", rel_z_move)
                elif name == 'joint_lift':
                    # 位置控制
                    joint_lift_position = joint_positions[i]
                    joint_lift_action = create_default_motor_msg()
                    joint_lift_action.mode = 1
                    joint_lift_action.pose.position.z = joint_lift_position# + (0.02 / len(joint_trajectory["position_trajectory"]))*cmd_idx
                    self.node.lift_publisher.publish(joint_lift_action)
                    # print("[debug]joint_lift_pos:", joint_lift_position)
                elif name == 'joint_arm_l2':
                    arm_pos += joint_positions[i]
                    arm_vel += joint_velocities[i]
                elif name == 'joint_arm_l1':
                    arm_pos += joint_positions[i]
                    arm_vel += joint_velocities[i]
                elif name == 'joint_arm_l0':
                    arm_pos += joint_positions[i]
                    arm_vel += joint_velocities[i]
                elif name == 'joint_yaw':
                    joint_yaw_position = joint_positions[i]
                    joint_yaw_action = create_default_motor_msg()
                    joint_yaw_action.pose.orientation.w, joint_yaw_action.pose.orientation.x, joint_yaw_action.pose.orientation.y, joint_yaw_action.pose.orientation.z = euler_to_quaternion(
                        0, 0, joint_yaw_position)
                    self.node.wrist_yaw_publisher.publish(joint_yaw_action)
                    # print("[debug]joint_yaw_pos:", joint_yaw_position)
                elif name == 'joint_pitch':
                    joint_pitch_position = joint_positions[i]
                    joint_pitch_action = create_default_motor_msg()
                    joint_pitch_action.pose.orientation.w, joint_pitch_action.pose.orientation.x, joint_pitch_action.pose.orientation.y, joint_pitch_action.pose.orientation.z = euler_to_quaternion(
                        0, 0, joint_pitch_position)
                    self.node.wrist_pitch_publisher.publish(joint_pitch_action)
                    # print("[debug]joint_pitch_pos:", joint_pitch_position)
                elif name == 'joint_roll':
                    joint_roll_position = joint_positions[i]
                    joint_roll_action = create_default_motor_msg()
                    joint_roll_action.pose.orientation.w, joint_roll_action.pose.orientation.x, joint_roll_action.pose.orientation.y, joint_roll_action.pose.orientation.z = euler_to_quaternion(
                        0, 0, joint_roll_position)
                    self.node.wrist_roll_publisher.publish(joint_roll_action)
                    # print("[debug]joint_roll_pos:", joint_roll_position)
            arm_action = create_default_motor_msg()
            arm_action.mode = 1
            arm_action.pose.position.x = arm_pos
            self.node.arm_publisher.publish(arm_action)
            # print("[debug]joint_arm_pos:", arm_pos)
            # print("[debug]joint_arm_vel:", arm_vel)

            cmd_idx += 1
            time.sleep(control_time) # 0.02为补偿
            end_time = time.time()
            total_time = end_time - start_time
            # print("用时：", total_time)
            # print("cmd_idx:", cmd_idx)

    def stop_trajectory(self):
        # stop motor with velocity mode
        # todo:当有关节采用速度控制时在这里做停止
        base_z_action = create_default_motor_msg()
        base_z_action.mode = 0
        base_z_action.vel.angular.z = 0.
        self.node.base_z_vel_publisher.publish(base_z_action)

        joint_lift_action = create_default_motor_msg()
        joint_lift_action.mode = 0
        joint_lift_action.vel.linear.z = 0.
        self.node.lift_publisher.publish(joint_lift_action)

        arm_action = create_default_motor_msg()
        arm_action.mode = 0
        arm_action.vel.linear.x = 0.
        self.node.arm_publisher.publish(arm_action)

    def execute_trajectory_close_loop(self, joint_trajectory):
        # todo:not implement yet
        print("close_loop_control not implement yet, please modify self.is_open_loop_control as True")
        pass


