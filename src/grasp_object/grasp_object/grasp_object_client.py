import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from grasp_get_interfaces.srv import GetPose
from gongga_interface.srv import GeneratePlan
from gongga_interface.action import FollowJointTrajectory     
from motor_interface.msg import VelAndPose
from motor_interface.msg import DebugData
from rclpy.action import ActionClient
from grasp_get_interfaces.srv import CommandService  # 假设你有一个自定义的 Command 服务
from sensor_msgs.msg import JointState  # 确保 JointState 已导入
import time
import json



class GraspObjectClient(Node):
    def __init__(self):
        super().__init__('grasp_object_client')

        # 创建服务客户端
        self.get_grasp_pose_client = self.create_client(GetPose, '/get_grasp_poses')
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
        self.grasp_pose = []
        self.task = ''
        self.task_status = 'waiting'
        self.task_reason = ''
        self.status_sent_count = 0  # 状态消息发送计数器
        self.task_command = ''
        # 定时器，用于定时发布状态信息
        self.timer = None
        # 心跳检测定时器
        self.heartbeat_subscription = self.create_subscription(String, '/brain/heartbeat', self.heartbeat_callback, 10)
        # 设定心跳时间限制
        self.heartbeat_received = True
        self.heartbeat_timer = self.create_timer(1.0, self.check_heartbeat)

        # 仿真修改项，发布末端轨迹到仿真平台
        self.eePosPublisher = self.create_publisher(DebugData, '/brain/ee_pos', 10)

    def task_command_callback(self, msg):
        """处理来自上层大脑的任务指令"""
        self.task_command = msg.data
        self.get_logger().info(f'收到任务指令: "{self.task_command}"')

        if self.task_status == 'working':
            self.get_logger().warn('任务正在进行中，忽略新的任务指令')
            return
        self.task = json.loads(self.task_command).get('cmd_type').lower()

        self.is_task_active = True
        self.task_status = 'working'
        self.reset_task()

    
    def reset_task(self):
        """重置任务状态并启动定时器发布状态"""
        self.status_sent_count = 0
        self.task_status = 'working'
        self.task_reason = ''
        self.grasp_pose = []

        if self.timer:
            self.timer.cancel()

        self.timer = self.create_timer(0.2, self.publish_status)

    def stop_task(self, stop_motion = True):
        """停止任务并发送停止指令"""
        self.is_task_active = False
        self.update_status(task = '', status='stopped', reason='任务被停止')
        # 发送停止指令给控制器
        if stop_motion:
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
        rclpy.spin_until_future_complete(self, future)
        self.check_driver_result(future)

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
    
    def handle_error(self, stage, error_message=""):
        """处理错误并停止状态更新"""
        self.get_logger().error(f"在 {stage} 阶段发生错误: {error_message}")
        self.update_status(task='', status='failed', reason=error_message)

    def update_status(self, task, status, reason):
        """更新并发布状态信息"""
        self.task = task
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

        self.update_status(self.task, self.task_status, self.task_reason)

    def stop_status_updates(self):
        """停止定时器发布状态"""
        if self.timer:
            self.timer.cancel()

    def get_grasp_pose(self):
        """请求抓取位姿"""
        if not self.get_grasp_pose_client.wait_for_service(timeout_sec=1.0):
            self.handle_error('获取抓取位姿', '服务不可用')
            return False

        request = GetPose.Request()
        request.calculate_grasp_poses = "calculate"
        request.cmd_json = self.task_command
        # 发起请求
        self.get_logger().info('请求目标位姿...')
        future = self.get_grasp_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        # 处理抓取位姿的结果
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'请求目标位姿时出错: {e}')
            return False
        if response.success:
            self.get_logger().info("获取目标位姿成功!")
            self.grasp_pose = response.pose
            return True
        else:
            self.handle_error('获取目标位姿', response.error_message)
            return False

    def generate_motion_plan(self):
        # 生成目标位姿的运动规划
        if not self.motion_generate_client.wait_for_service(timeout_sec=10.0):
            self.handle_error('运动规划', '服务不可用')
            return
        # 发起请求
        request = GeneratePlan.Request()
        request.task = "get_joint_states"
        future = self.motion_generate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.2)
        request.task = "generate_motion"
        request.target_pose = self.grasp_pose
        self.get_logger().info(f'发送目标位姿给运动规划: {request.target_pose}')

        future = self.motion_generate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        # 处理运动规划的结果
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'运动规划请求时出错: {e}')
            return False
        if response.is_success:
            self.get_logger().info("获取运动轨迹成功!")
            self.joint_trajectory = response.joint_trajectory
            # 记录末端轨迹
            self.ee_pos_x = response.ee_pos_x
            self.ee_pos_y = response.ee_pos_y
            self.ee_pos_z = response.ee_pos_z
        else:
            self.get_logger().error(f'运动规划失败: {response.failure_reason}')
        return response.is_success


    def send_joint_trajectory(self):
        # 发送关节轨迹给下层控制器
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = self.joint_trajectory
        goal_msg.control_rate = 5  #Hz
        # 检查服务器是否可用
        self.get_logger().info("等待关节轨迹控制服务...")
        if not self.gongga_driver_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("未连接到关节轨迹控制服务")
            self.handle_error('关节轨迹发送', "未连接到关节轨迹控制服务")
            return
        # 异步发送目标，并展示进度
        self.get_logger().info("发送关节轨迹目标")
        future = self.gongga_driver_client.send_goal_async(
            goal_msg, feedback_callback=self.gongga_driver_feedback_callback
        )
        rclpy.spin_until_future_complete(self, future)
        rst = self.check_driver_result(future)
        return rst
    
    def check_driver_result(self, future):
        # 运动完毕，处理结果
        self.get_logger().info("接收到关节轨迹控制器响应")
        try:
            # 获取服务响应结果
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("目标未被接收")
                self.handle_error('关节轨迹发送', "目标未被接收")
                return
        except Exception as e:
            self.get_logger().error(f"发送关节轨迹目标时出错: {e}")
            self.handle_error('关节轨迹发送', str(e))
        # 等待结果
        self.get_logger().info("等待关节轨迹执行结果...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        # 处理关节轨迹执行结果
        try:
            result = result_future.result().result
        except Exception as e:
            self.get_logger().error(f"获取关节轨迹执行结果时出错: {e}")
            self.handle_error('关节轨迹执行', str(e))
        if result.is_success:
            self.get_logger().info("轨迹执行成功!")
        else:
            self.get_logger().error(f"轨迹执行失败: {result.failure_reason}")
            self.handle_error('关节轨迹执行', result.failure_reason)
        return result.is_success

    def gongga_driver_feedback_callback(self, feedback_msg):
        """反馈关节轨迹执行进度"""
        progress = feedback_msg.feedback.progress_percentage
        self.get_logger().info(f"轨迹执行进度: {progress*100}%")

    
    def move_gripper(self, cmd, pos=0):
        match cmd:
            case 'open':
                gripper_action = VelAndPose()
                gripper_action.mode = 1
                gripper_action.pose.orientation.x = 0.0
                gripper_action.pose.orientation.y = 0.0
                gripper_action.pose.orientation.z = 1.0
                gripper_action.pose.orientation.w = 0.0
                self.get_logger().info('打开夹爪')
                self.gripper_publisher.publish(gripper_action)
            case 'close':
                gripper_action = VelAndPose()
                gripper_action.mode = 1
                gripper_action.pose.orientation.x = 0.0
                gripper_action.pose.orientation.y = 0.0
                gripper_action.pose.orientation.z = -0.766
                gripper_action.pose.orientation.w = 0.643
                self.get_logger().info('闭合夹爪')
                self.gripper_publisher.publish(gripper_action)
            case 'lift':
                lift_action = VelAndPose()
                lift_action.mode = 1
                lift_action.pose.position.z = pos
                self.get_logger().info(f'运动夹爪至{pos}')
                self.lift_publisher.publish(lift_action)
            case _:
                self.get_logger().error('未知夹爪命令')

    def execute_task(self):
        match self.task:
            case 'pickup':
                self.get_logger().info('开始执行抓取任务...')
                self.execute_pickup_task()
            case 'release':
                self.get_logger().info('开始执行放物任务...')
                self.execute_release_task()
            case 'stop':
                self.get_logger().info('接收到停止任务指令，停止所有工作...')
                self.stop_task()
            case _:
                self.task_status = 'stopped'
                self.is_task_active = False
        
    
    def execute_pickup_task(self):
        # 执行抓取任务，任务流程：
        # 获取抓取点位；进行运动规划；执行运动；收拢夹爪并抬起；
        if not self.is_task_active:
            return
        self.move_gripper('lift', 0.85)
        self.move_gripper('open')
        time.sleep(3)
        # 根据rbg和深度图获取抓取点位
        issuccess = self.get_grasp_pose()
        if issuccess == True:
            issuccess = self.generate_motion_plan()
        else:
            self.stop_task()
            return
        # 进行运动规划
        if issuccess == True:
            self.pub_debug()
            issuccess = self.send_joint_trajectory()
        else:
            self.stop_task()
            return
        # 收拢夹爪并抬起
        if issuccess == True:
            self.move_gripper('close')
            time.sleep(3)
            self.move_gripper('lift', 0.85)
            time.sleep(3)
            self.stop_motion()
            time.sleep(3)
            self.get_logger().info('抓取完成')
            self.update_status(task='', status='succeeded', reason='任务完成')
        else:
            self.stop_task()
            return
        
    def execute_release_task(self):
        # 执行放物任务，任务流程：
        # 获取放物点位；进行运动规划；执行运动；打开夹爪并抬起；
        if not self.is_task_active:
            return
        self.move_gripper('lift', 0.85)
        time.sleep(3)
        # 根据rbg和深度图获取抓取点位
        issuccess = self.get_grasp_pose()
        if issuccess == True:
            issuccess = self.generate_motion_plan()
        else:
            self.stop_task()
            return
        if issuccess == True:
            self.pub_debug()
            issuccess = self.send_joint_trajectory()
        else: 
            self.stop_task(stop_motion=False)
            return
        # 放开夹爪并抬起
        if issuccess == True:
            self.move_gripper('open')
            time.sleep(3)
            self.move_gripper('lift', 0.85)
            time.sleep(3)
            self.get_logger().info('放置完成')
            self.update_status(task='', status='succeeded', reason='任务完成')
            self.stop_motion()
        else:
            self.stop_task()
            return
    
    def pub_debug(self):
        debugData = DebugData()
        debugData.ee_pos_x = self.ee_pos_x
        debugData.ee_pos_y = self.ee_pos_y
        debugData.ee_pos_z = self.ee_pos_z
        debugData.rgb_url = json.loads(self.task_command).get("image_rgb_url")
        debugData.depth_url = json.loads(self.task_command).get("image_depth_url")
        self.eePosPublisher.publish(debugData)

def main(args=None):
    rclpy.init(args=args)
    grasp_object_client = GraspObjectClient()
    try:
        while rclpy.ok():
            rclpy.spin_once(grasp_object_client, timeout_sec=0.05)
            grasp_object_client.execute_task()
    except KeyboardInterrupt:
        grasp_object_client.get_logger().info('用户关闭节点...')
    finally:
        grasp_object_client.destroy_node()

if __name__ == '__main__':
    main()
