import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from grasp_get_interfaces.srv import GetPose
from gongga_interface.srv import GeneratePlan
from gongga_interface.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient

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

        # 初始化任务状态
        self.is_task_active = False
        self.grasp_poses = []
        self.is_task_complete = False
        self.task_status = 'waiting'
        self.task_reason = ''
        self.failed_at_last_pose = False
        self.status_sent_count = 0  # 状态消息发送计数器

        # 定时器，用于定时发布状态信息
        self.timer = None

    def task_command_callback(self, msg):
        """处理来自上层大脑的任务指令"""
        task_command = msg.data
        self.get_logger().info(f'收到任务指令: "{task_command}"')

        if self.task_status == 'working':
            self.get_logger().warn('任务正在进行中，忽略新的任务指令')
            return

        if 'pickup' in task_command.lower():
            self.get_logger().info('开始执行抓取任务...')
            self.is_task_active = True
            self.task_status = 'working'
            self.reset_task()
            self.execute_grasp_task()

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

        self.get_logger().info('请求抓取位姿...')
        future = self.get_grasp_poses_client.call_async(request)
        future.add_done_callback(self.grasp_poses_callback)

    def grasp_poses_callback(self, future):
        """处理抓取位姿的响应"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'收到 {len(response.poses)} 个抓取位姿')
                self.grasp_poses = response.poses
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
        status_msg.data = f"pickup,{status},{reason}"
        self.status_publisher.publish(status_msg)

        if status in ['succeeded', 'failed']:
            self.status_sent_count += 1
            if self.status_sent_count >= 3:
                self.stop_status_updates()

    def publish_status(self):
        """定期发布任务状态"""
        if self.task_status in ['succeeded', 'failed'] and self.status_sent_count >= 3:
            return

        self.update_status(self.task_status, self.task_reason)

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
                self.handle_error('运动规划', "机器人抓取不到物体")
            return

        pose = self.grasp_poses[self.current_pose_index]
        self.get_logger().info(f'发送抓取位姿 {self.current_pose_index + 1} 给运动规划...')
        self.generate_motion_plan(pose)

    def generate_motion_plan(self, target_pose):
        """生成目标位姿的运动规划"""
        if not self.motion_generate_client.wait_for_service(timeout_sec=10.0):
            self.handle_error('运动规划', '服务不可用')
            return

        request = GeneratePlan.Request()
        request.target_pose = target_pose
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

    def stop_status_updates(self):
        """停止定时器发布状态"""
        if self.timer:
            self.timer.cancel()

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
                self.get_logger().info("轨迹执行成功!")
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
