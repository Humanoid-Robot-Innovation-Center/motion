import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from grasp_get_interfaces.srv import GetPose
from gongga_interface.srv import GeneratePlan
from gongga_interface.action import FollowJointTrajectory     
from motor_interface.msg import VelAndPose
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.service import Service
from grasp_get_interfaces.srv import CommandService  # 假设你有一个自定义的 Command 服务
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState  # 确保 JointState 已导入
import time
import json

class GraspObjectClient(Node):
    def __init__(self):
        super().__init__('grasp_object_client')

        # 创建服务客户端
        self.get_grasp_poses_client = self.create_client(GetPose, '/get_grasp_poses')
        self.motion_generate_client = self.create_client(GeneratePlan, '/gongga_planner/motion_generator')

        # 创建 action client，连接到 FollowJointTrajectory action
        self.gongga_driver_client = ActionClient(self, FollowJointTrajectory, '/gongga_controller/follow_joint_trajectory')

        # 创建订阅者，监听上层大脑发布的任务指令
        self.create_subscription(String, '/brain/commands', self.task_command_callback, 10)

        # 创建状态反馈发布者
        self.status_publisher = self.create_publisher(String, '/brain/feedback', 10)
        self.lift_publisher = self.create_publisher(VelAndPose, "cmd_lift", 10)
        self.gripper_publisher = self.create_publisher(VelAndPose, "gripper_pull", 10)
        
        self.camera_publisher = self.create_publisher(VelAndPose, "/camera_pitch", 10)
        self.wrist_publisher = self.create_publisher(VelAndPose, "/wrist_pitch", 10)

        # 初始化任务状态
        self.is_task_active = False
        self.grasp_poses = []
        self.is_task_complete = False
        self.task_status = 'waiting'
        self.task_reason = ''
        self.failed_at_last_pose = False
        self.status_sent_count = 0  # 状态消息发送计数器
        self.task_command = ''

        # 定时器，用于定时发布状态信息
        self.timer = None

        # 创建自定义服务，监听任务指令
        self.command_service = self.create_service(CommandService, '/brain/command_service', self.command_service_callback)

        # 心跳检测定时器
        self.heartbeat_subscription = self.create_subscription(String, '/brain/heartbeat', self.heartbeat_callback, 10)

        # 设定心跳时间限制
        self.heartbeat_received = True
        self.heartbeat_timer = self.create_timer(1.0, self.check_heartbeat)

    def task_command_callback(self, msg):
        """处理来自上层大脑的任务指令"""
        self.task_command = msg.data
        self.get_logger().info(f'收到任务指令: "{self.task_command}"')

        if self.task_status == 'working':
            self.get_logger().warn('任务正在进行中，忽略新的任务指令')
            return
        
        task = json.loads(self.task_command).get('cmd_type').lower()

        cmd_list = ['pickup', 'open', 'close', 'turn_on', 'turn_off']
        if task in cmd_list:
            self.get_logger().info('开始执行抓取任务...')
            self.is_task_active = True
            self.task_status = 'working'
            self.reset_task()

            # self.get_logger().info('开始执行抓取任务...')
            # cameara_action = VelAndPose()
            # cameara_action.mode = 1
            # cameara_action.pose.orientation.x = 0.0
            # cameara_action.pose.orientation.y = 0.0
            # cameara_action.pose.orientation.z = -0.500
            # cameara_action.pose.orientation.w = 0.866
            # self.get_logger().info('调整相机')
            # self.camera_publisher.publish(cameara_action)
            # wrist_action = VelAndPose()
            # wrist_action.mode = 1
            # wrist_action.pose.orientation.x = 0.0
            # wrist_action.pose.orientation.y = 0.0
            # wrist_action.pose.orientation.z = 0.0
            # wrist_action.pose.orientation.w = 1.0
            # self.get_logger().info('调整手腕')
            # self.wrist_publisher.publish(wrist_action)
            # time.sleep(3)

            self.execute_grasp_task()
        elif 'stop' in task:
            self.get_logger().info('接收到停止任务指令，停止所有工作...')
            self.stop_task()

    def reset_task(self):
        """重置任务状态并启动定时器发布状态"""
        self.is_task_complete = False
        self.failed_at_last_pose = False
        self.status_sent_count = 0
        self.task_status = 'working'
        self.task_reason = ''
        self.grasp_poses = []

        if self.timer:
            self.timer.cancel()

        self.timer = self.create_timer(0.2, self.publish_status)

    def stop_task(self):
        """停止任务并发送停止指令"""
        self.is_task_active = False
        self.task_status = 'stopped'
        self.update_status(self.task_status, '任务被停止')

        # 发送停止指令给控制器
        self.stop_motion()

        # 停止定时器发布状态
        self.stop_status_updates()

    def stop_motion(self):
        """发送停止指令给关节控制器"""
        empty_trajectory = JointState()

        # 清空必要的字段，确保符合类型要求
        empty_trajectory.header.stamp.sec = 0
        empty_trajectory.header.stamp.nanosec = 0
        empty_trajectory.header.frame_id = ""  # 清空 frame_id
        empty_trajectory.name = []  # 清空关节名称
        empty_trajectory.position = []  # 清空位置
        empty_trajectory.velocity = []  # 清空速度
        empty_trajectory.effort = []  # 清空力矩

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = empty_trajectory
        goal_msg.control_rate = 10

        self.get_logger().info("发送停止指令给关节轨迹控制器...")
        future = self.gongga_driver_client.send_goal_async(goal_msg)
        future.add_done_callback(self.gongga_driver_callback)

    def command_service_callback(self, request, response):
        """处理任务指令服务请求"""
        command_type = request.command_type
        target_cmd = request.target_cmd
        self.get_logger().info(f"接收到指令: {command_type} {target_cmd}")

        if command_type == "stop":
            self.stop_task()
            response.result = "success"
            response.reason = "任务已停止"
        else:
            response.result = "error"
            response.reason = "未知指令"
        return response

    def heartbeat_callback(self, msg):
        """监听心跳消息"""
        self.heartbeat_received = True

    def check_heartbeat(self):
        """检查心跳是否超时"""
        if self.task_status == 'working' and not self.heartbeat_received:
            self.get_logger().info("没有收到心跳消息，停止任务...")
            self.stop_task()

        # 重置心跳检测
        self.heartbeat_received = False
    def execute_grasp_task(self):
        """执行抓取任务，首先请求抓取位姿"""
        if not self.is_task_active:
            return
        
        self.get_grasp_poses()

    def get_grasp_poses(self):
        """请求抓取位姿"""
        if not self.get_grasp_poses_client.wait_for_service(timeout_sec=1.0):
            self.handle_error('获取抓取位姿', '服务不可用')
            return

        request = GetPose.Request()
        request.calculate_grasp_poses = "calculate"
        request.cmd_json = self.task_command
        
        self.get_logger().info('请求抓取位姿...')
        future = self.get_grasp_poses_client.call_async(request)
        future.add_done_callback(self.grasp_poses_callback)

    def grasp_poses_callback(self, future):
        """处理抓取位姿的响应"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'收到 {len(response.poses)} 个抓取位姿')
                gripper_action = VelAndPose()
                gripper_action.mode = 1
                gripper_action.pose.orientation.x = 0.0
                gripper_action.pose.orientation.y = 0.0
                gripper_action.pose.orientation.z = 1.0
                gripper_action.pose.orientation.w = 0.0
                self.get_logger().info('打开夹爪')

                self.gripper_publisher.publish(gripper_action)
                self.grasp_poses = response.poses
                # self.marker = response.marker
                self.send_to_motion_planner()
            else:
                self.handle_error('获取抓取位姿', response.error_message)
        except Exception as e:
            self.handle_error('获取抓取位姿', str(e))

    def handle_error(self, stage, error_message=""):
        """处理错误并停止状态更新"""
        self.get_logger().error(f"在 {stage} 阶段发生错误: {error_message}")
        self.update_status('failed', error_message)

    def update_status(self, status, reason):
        """更新并发布状态信息"""
        self.task_status = status
        self.task_reason = reason
        status_msg = String()
        # status_msg.data = f"{self.task_command},{status},{reason},/brain/command_service"
        data = {
            "command":self.task_command,
            "status":status,
            "reason":reason,
            "command_topic":"/brain/command_service"
        }
        status_msg.data = json.dumps(data,ensure_ascii=False)
        self.status_publisher.publish(status_msg)

        if status in ['succeeded', 'failed', 'stopped']:
            self.status_sent_count += 1
            if self.status_sent_count >= 3:
                self.stop_status_updates()

    def publish_status(self):
        """定期发布任务状态"""
        if self.task_status in ['succeeded', 'failed', 'stopped'] and self.status_sent_count >= 3:
            return

        self.update_status(self.task_status, self.task_reason)

    def stop_status_updates(self):
        """停止定时器发布状态"""
        if self.timer:
            self.timer.cancel()
    def send_to_motion_planner(self):
        """将抓取位姿逐个发送给运动规划服务"""
        self.failed_at_last_pose = False
        self.pending_poses_count = len(self.grasp_poses)
        self.current_pose_index = 0  # 初始化当前位姿索引

        # 发送第一个位姿
        self.send_next_pose()

    def send_next_pose(self):
        """发送当前索引指向的位姿给运动规划服务"""
        if self.current_pose_index >= len(self.grasp_poses):
            # 如果所有位姿均已处理，且没有成功的规划，则视为抓取失败
            if not self.is_task_complete:
                print(self.current_pose_index)
                self.handle_error('运动规划', "机器人抓取不到物体")
            return

        pose = self.grasp_poses[self.current_pose_index]
        # target_marker = self.marker

        self.get_logger().info(f'发送抓取位姿 {self.current_pose_index + 1} 给运动规划...')
        self.generate_motion_plan(pose)

    def generate_motion_plan(self, target_pose):
        """生成目标位姿的运动规划"""
        if not self.motion_generate_client.wait_for_service(timeout_sec=10.0):
            self.handle_error('运动规划', '服务不可用')
            return

        request = GeneratePlan.Request()
        request.target_pose = target_pose
        # request.marker = marker
        self.get_logger().info(f'发送目标位姿给运动规划: {target_pose}')
        future = self.motion_generate_client.call_async(request)
        future.add_done_callback(self.motion_plan_callback)

    def motion_plan_callback(self, future):
        """处理运动规划响应"""
        try:
            response = future.result()
            if response.is_success:
                self.get_logger().info('运动规划成功!')
                self.is_task_complete = True  # 任务完成
                self.send_joint_trajectory(response.joint_trajectory)
                return  # 成功则直接返回，不再发送后续位姿

            else:
                self.get_logger().error(f'运动规划失败: {response.failure_reason}')
                self.pending_poses_count -= 1  # 减少待处理位姿数

        except Exception as e:
            self.get_logger().error(f'运动规划请求时出错: {e}')
            self.pending_poses_count -= 1  # 减少待处理位姿数

        # 更新索引到下一个位姿并尝试再次发送
        self.current_pose_index += 1
        self.send_next_pose()


    def send_joint_trajectory(self, joint_trajectory):
        """发送关节轨迹给下层控制器"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = joint_trajectory
        goal_msg.control_rate = 10

        # 检查服务器是否可用
        self.get_logger().info("等待关节轨迹控制服务...")
        if not self.gongga_driver_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("未连接到关节轨迹控制服务")
            self.handle_error('关节轨迹发送', "未连接到关节轨迹控制服务")
            return

        # 异步发送目标
        self.get_logger().info("发送关节轨迹目标")
        future = self.gongga_driver_client.send_goal_async(
            goal_msg, feedback_callback=self.gongga_driver_feedback_callback
        )
        future.add_done_callback(self.gongga_driver_callback)

    def gongga_driver_feedback_callback(self, feedback_msg):
        """反馈关节轨迹执行进度"""
        progress = feedback_msg.feedback.progress_percentage
        self.get_logger().info(f"轨迹执行进度: {progress}%")


    def gongga_driver_callback(self, future):
        """处理下层控制器响应"""
        self.get_logger().info("接收到关节轨迹控制器响应")
        try:
            # 获取服务响应结果
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("目标未被接收")
                self.handle_error('关节轨迹发送', "目标未被接收")
                return

            # 异步等待结果
            self.get_logger().info("等待关节轨迹执行结果...")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.gongga_driver_result_callback)

        except Exception as e:
            self.get_logger().error(f"发送关节轨迹目标时出错: {e}")
            self.handle_error('关节轨迹发送', str(e))

    def gongga_driver_result_callback(self, future):
        """处理关节轨迹执行结果"""
        try:
            result = future.result().result
            if result.is_success:
                self.get_logger().info("轨迹执行成功，开始抓取!")
                gripper_action = VelAndPose()
                gripper_action.mode = 1
                gripper_action.pose.orientation.x = 0.0
                gripper_action.pose.orientation.y = 0.0
                gripper_action.pose.orientation.z = -0.766
                gripper_action.pose.orientation.w = 0.643
                self.get_logger().info('闭合夹爪')

                self.gripper_publisher.publish(gripper_action)
                time.sleep(3.0)

                lift_action = VelAndPose()
                lift_action.mode = 1
                lift_action.pose.position.z = 0.85
                self.lift_publisher.publish(lift_action)
                self.get_logger().info('抬升夹爪')
                time.sleep(3.0)
                self.update_status('succeeded', '任务完成')
            else:
                self.get_logger().error(f"轨迹执行失败: {result.failure_reason}")
                self.handle_error('关节轨迹发送', result.failure_reason)
        except Exception as e:
            self.get_logger().error(f"获取关节轨迹执行结果时出错: {e}")
            self.handle_error('关节轨迹执行', str(e))

def main(args=None):
    rclpy.init(args=args)
    grasp_object_client = GraspObjectClient()
    rclpy.spin(grasp_object_client)

if __name__ == '__main__':
    main()
