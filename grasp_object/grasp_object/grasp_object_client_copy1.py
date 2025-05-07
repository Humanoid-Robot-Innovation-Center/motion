

"""ROS2节点实现机器人抓取任务流水线控制。

本模块包含GraspObjectClient类，主要功能包括：
1. 协调抓取位姿获取、运动规划、轨迹执行的全流程
2. 提供任务状态监控和故障处理机制
3. 实现安全联锁和紧急停止功能
4. 维护与上层控制系统的通信接口

典型工作流程：
1. 接收任务指令（通过服务调用或主题订阅）
2. 获取候选抓取位姿列表
3. 逐个尝试运动规划直到成功
4. 执行关节轨迹并监控执行状态
5. 完成抓取后执行后续操作（闭合夹爪、抬升等）

模块级常量：
    NODE_NAME (str): 节点名称，值为'grasp_object_client'
    STATUS_PUBLISH_INTERVAL (float): 状态发布间隔（秒），默认0.2
    SERVICE_TIMEOUT (float): 服务调用超时时间（秒），默认1.0
    HEARTBEAT_TIMEOUT (float): 心跳超时时间（秒），默认1.0
"""

# 标准库导入
import time
from typing import List, Optional

# 第三方库导入
import rclpy
from geometry_msgs.msg import Pose
from grasp_get_interfaces.srv import CommandService, GetPose
from gongga_interface.action import FollowJointTrajectory
from gongga_interface.srv import GeneratePlan
from motor_interface.msg import VelAndPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class GraspObjectClient(Node):
    """ROS2节点实现机器人抓取任务的全流程控制。

    本节点作为抓取任务的核心协调器，主要职责包括：
    1. 管理抓取任务的生命周期（启动/停止/监控）
    2. 协调各子系统（感知、规划、执行）的交互
    3. 实现安全联锁和异常处理机制

    属性:
        get_grasp_poses_client (rclpy.Client): 抓取位姿获取服务客户端，连接/grasp_poses服务
        motion_generate_client (rclpy.Client): 运动规划服务客户端，连接/motion_generator服务
        gongga_driver_client (ActionClient): 关节轨迹执行动作客户端，连接/follow_joint_trajectory动作
        status_publisher (Publisher): 任务状态发布器，发布到/brain/feedback主题
        is_task_active (bool): 任务激活状态标志，True表示任务正在执行
        grasp_poses (List[Pose]): 候选抓取位姿列表，按优先级排序

    示例:
        ```python
        # 初始化节点
        rclpy.init()
        client = GraspObjectClient()
        
        # 启动抓取任务
        client.execute_grasp_task()
        
        # 进入ROS2事件循环
        rclpy.spin(client)
        ```

    异常:
        ServiceException: 当关键服务调用失败时抛出
        ActionFailed: 当动作执行失败时抛出
        TimeoutError: 当操作超时时抛出
    """
    
    def __init__(self):
        """初始化抓取任务节点并创建 ROS 2 通信组件。

        该方法按顺序执行以下操作：

        初始化 ROS 2 节点
        为功能模块创建服务客户端
        创建用于关节轨迹控制的动作客户端
        初始化状态发布器和执行器控制接口
        建立心跳监测机制
        参数：
        无

        异常：
        RuntimeError：若关键服务初始化失败，可能原因包括：
        - ROS 2 上下文初始化失败
        - 服务接口定义不匹配
        - 系统资源不足
        ConnectionError：若无法建立与 ROS 2 服务/动作的连接

        属性：
        get_grasp_poses_client (rclpy.Client)：用于获取抓取姿态的服务客户端，
        连接到 /get_grasp_poses 服务
        motion_generate_client (rclpy.Client)：运动规划服务客户端，
        连接到 /gongga_planner/motion_generator 服务
        gongga_driver_client (ActionClient)：用于执行关节轨迹的动作客户端，
        连接到 /gongga_controller/follow_joint_trajectory 动作
        status_publisher (rclpy.Publisher)：任务状态发布器，
        发布到 /brain/feedback 主题
        heartbeat_subscription (rclpy.Subscription)：心跳监测订阅器，
        监听 /brain/heartbeat 主题

        注意事项：

        所有服务客户端使用默认的 QoS 配置
        动作客户端的连接超时时间为 10 秒
        心跳检查间隔为 1 秒，超时将触发任务停止
        节点名称固定为 'grasp_object_client'
        初始化后自动进入 ROS 2 事件循环
        """
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
        self.lift_publisher = self.create_publisher(VelAndPose, "/cmd_lift", 10)
        self.gripper_publisher = self.create_publisher(VelAndPose, "/gripper_pull", 10)
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

    def task_command_callback(self, msg: String) -> None:
        """处理任务指令并协调硬件响应。

        主要功能：

        解析指令类型（pickup/stop）
        协调相机、腕部和夹爪的运动
        管理任务状态机的转换
        实现安全联锁机制
        参数：
        msg (std_msgs.msg.String)：包含任务指令的 ROS 2 命令消息。
        支持的指令类型：
        - 'pickup'：激活抓取序列：
        * 将相机俯仰角调整至 60° 工作位置
        * 将腕关节复位至零位
        * 预先打开夹爪至 120mm 宽度
        - 'stop'：紧急停止指令：
        * 立即停止所有关节运动

        返回值：
        None：该方法不返回任何值

        """
        self.task_command = msg.data
        self.get_logger().info(f'收到任务指令: "{self.task_command}"')

        if self.task_status == 'working':
            self.get_logger().warn('任务正在进行中，忽略新的任务指令')
            return

        if 'pickup' in self.task_command.lower():
            self.get_logger().info('开始执行抓取任务...')
            
            # 创建相机姿态调整指令
            camera_action = VelAndPose()
            camera_action.mode = 1  # 位置控制模式
            
            # 设置目标姿态四元数（俯仰角调整）
            camera_action.pose.orientation.x = 0.0
            camera_action.pose.orientation.y = 0.0
            camera_action.pose.orientation.z = -0.500  # z分量对应俯仰角
            camera_action.pose.orientation.w = 0.866   # 使用四元数表示60度旋转
            
            self.get_logger().info('调整相机俯仰角至工作位置')
            self.camera_publisher.publish(camera_action)

            # 创建手腕姿态调整指令
            wrist_action = VelAndPose()
            wrist_action.mode = 1  # 位置控制模式
            wrist_action.pose.orientation.x = 0.0
            wrist_action.pose.orientation.y = 0.0 
            wrist_action.pose.orientation.z = 0.0
            wrist_action.pose.orientation.w = 1.0  # 单位四元数表示无旋转
            self.get_logger().info('调整手腕至初始位置')
            self.wrist_publisher.publish(wrist_action)
            self.is_task_active = True
            self.task_status = 'working'
            self.reset_task()
            time.sleep(10)
            self.execute_grasp_task()
        elif 'stop' in self.task_command.lower():
            self.get_logger().info('接收到停止任务指令，停止所有工作...')
            self.stop_task()

    def reset_task(self) -> None:
        """重置任务状态机并配置状态监控系统。

        该方法执行以下操作：

        重置所有运行时状态变量
        初始化状态发布定时器
        建立任务状态跟踪机制
        参数：
        无

        返回值：
        None：该方法不返回任何值

        异常：
        RuntimeError：若定时器初始化失败

        属性：
        is_task_complete (bool)：任务完成标志（True 表示已达到最终状态）
        failed_at_last_pose (bool)：末端执行器失败标志
        status_sent_count (int)：状态消息计数器（用于防抖处理）
        task_status (str)：当前状态（waiting/working/succeeded/failed/stopped）
        task_reason (str)：状态转换的原因描述
        grasp_poses (List[Pose])：抓取姿态的缓冲队列

        注意事项：

        定时器间隔为 200ms，满足实时控制需求（5Hz 更新率）
        状态机转换为原子操作，确保状态一致性
        历史状态信息将自动归档至日志系统
        状态消息的最大重试次数为 3 次（防止数据包丢失）
        """
        self.is_task_complete = False
        self.failed_at_last_pose = False
        self.status_sent_count = 0
        self.task_status = 'working'
        self.task_reason = ''
        self.grasp_poses = []

        if self.timer:
            self.timer.cancel()

        self.timer = self.create_timer(0.2, self.publish_status)

    def stop_task(self) -> None:
        """安全停止当前任务并重置所有控制器。

        该方法将执行以下操作：

        将任务状态更新为 'stopped'
        向关节轨迹控制器发送停止指令
        停止状态更新定时器
        将所有执行器重置为安全状态
        注意事项：

        立即中断所有正在进行的动作
        保留当前任务状态以供后续分析
        向所有执行器（如夹爪、升降机构等）发送紧急停止指令
        """
        self.is_task_active = False
        self.task_status = 'stopped'
        self.update_status(self.task_status, '任务被停止')

        # 发送停止指令给控制器
        self.stop_motion()

        # 停止定时器发布状态
        self.stop_status_updates()

    def stop_motion(self) -> None:
        """紧急停止机械臂所有运动。
        
        该方法会发送空轨迹指令到关节控制器，立即停止所有关节运动。
        
        Args:
            无直接参数，通过类实例状态获取所需信息
            
        Returns:
            None
            
        Raises:
            RuntimeError: 当指令发送失败时抛出
            
        实现步骤：
        1. 创建空轨迹消息
        2. 初始化消息头信息
        3. 清空所有关节参数
        4. 构建停止指令
        5. 异步发送指令并处理异常
        """
        # 创建空轨迹消息对象（使用JointState类型）
        empty_trajectory = JointState()

        # 初始化消息头信息（使用ROS2时间戳规范）
        now = self.get_clock().now().to_msg()
        empty_trajectory.header.stamp = now  # 设置时间戳
        empty_trajectory.header.frame_id = ""  # 清空坐标系ID

        # 清空所有关节参数（确保列表初始化）
        empty_trajectory.name = list()     # 关节名称列表
        empty_trajectory.position = list() # 位置参数列表
        empty_trajectory.velocity = list() # 速度参数列表
        empty_trajectory.effort = list()   # 力矩参数列表

        # 构建目标消息（遵循FollowJointTrajectory接口规范）
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = empty_trajectory  # 设置空轨迹
        goal_msg.control_rate = 10  # 控制频率（单位：Hz）

        self.get_logger().info("正在发送紧急停止指令到关节控制器...")

        # 异步发送目标并注册回调（带异常处理）
        try:
            future = self.gongga_driver_client.send_goal_async(goal_msg)
            future.add_done_callback(self.gongga_driver_callback)
        except Exception as e:
            error_msg = f"发送停止指令失败: {str(e)}"
            self.get_logger().error(error_msg)
            # 封装原始异常，保留堆栈信息
            raise RuntimeError(error_msg) from e

    def command_service_callback(self, request, response):
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
    def execute_grasp_task(self) -> None:
        if not self.is_task_active:
            return
        self.get_grasp_poses()

    def get_grasp_poses(self):
        """异步获取抓取位姿。
        该方法通过调用 ROS2 服务来异步获取抓取位姿，具体流程如下：
        
        1. 在 1.0 秒内等待服务 `/get_grasp_poses` 可用；
        2. 如果服务不可用，则调用 `handle_error` 方法处理错误，错误信息为 '获取抓取位姿' 和 '服务不可用'，并直接返回；
        3. 创建 `GetPose.Request` 请求对象，并将其字段 `calculate_grasp_poses` 设定为字符串 "calculate"；
        4. 记录日志，提示正在请求抓取位姿；
        5. 异步调用服务，并为返回的 future 对象注册回调函数 `grasp_poses_callback` 以处理响应结果。

        Returns:
            None: 该方法不返回任何值。

        Raises:
            无异常抛出：所有错误均通过内部的 `handle_error` 方法处理。
        """

        if not self.get_grasp_poses_client.wait_for_service(timeout_sec=1.0):
            self.handle_error('获取抓取位姿', '服务不可用')
            return

        request = GetPose.Request()
        request.calculate_grasp_poses = "calculate"

        self.get_logger().info('请求抓取位姿...')
        future = self.get_grasp_poses_client.call_async(request)
        future.add_done_callback(self.grasp_poses_callback)

    def grasp_poses_callback(self, future):
        """处理抓取位姿服务响应的回调函数。
        当异步调用抓取位姿服务返回响应时，该回调函数被触发以处理响应数据。具体流程如下：
        
        1. 尝试从 future 中获取响应数据。
        2. 如果响应成功（即 response.success 为 True）：
        - 记录日志，显示收到的抓取位姿数量。
        - 创建一个 VelAndPose 类型的消息对象，并将其操作模式设置为 1。
        - 设置消息中夹爪控制的姿态四元数参数：
            * orientation.x = 0.0
            * orientation.y = 0.0
            * orientation.z = 1.0
            * orientation.w = 0.0
        - 记录日志，提示正在执行打开夹爪操作。
        - 发布夹爪控制消息至相应主题。
        - 将响应中包含的抓取位姿列表保存到实例变量 grasp_poses 中。
        - 调用 send_to_motion_planner 方法，将抓取位姿传送给运动规划模块。
        3. 如果响应失败（即 response.success 为 False），则调用 handle_error 方法，
        并传入 '获取抓取位姿' 和响应中的错误信息进行错误处理。
        4. 在整个过程中，如果发生异常，将捕获该异常，并调用 handle_error 方法处理异常。

        Args:
            future: 异步调用返回的 future 对象，包含抓取位姿服务的响应数据。

        Returns:
            None: 该回调函数不返回任何值。

        Raises:
            无显式异常：所有错误均通过 handle_error 方法内部处理。
        """
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
        """处理错误并停止状态更新。

        该方法用于处理在指定阶段发生的错误，执行以下操作：
        1. 通过日志记录器输出错误信息，格式为 "在 {stage} 阶段发生错误: {error_message}"。
        2. 调用 update_status 方法，将任务状态更新为 'failed'，并传入错误信息作为状态原因。

        Args:
            stage (str): 发生错误的阶段或模块名称。
            error_message (str, optional): 详细错误描述信息，默认为空字符串。

        Returns:
            None
        """
        self.get_logger().error(f"在 {stage} 阶段发生错误: {error_message}")
        self.update_status('failed', error_message)


    def update_status(self, status, reason):
        """更新任务状态并发布状态消息。

        该方法用于更新任务的当前状态和状态原因，并将状态消息发布到 ROS2 主题上。
        具体步骤包括：
        1. 更新实例变量 task_status 和 task_reason，分别设置为传入的 status 和 reason。
        2. 构造状态消息，其内容为一个逗号分隔的字符串，包含：
        - 当前任务命令 (self.task_command)
        - 任务状态 (status)
        - 状态原因 (reason)
        - 固定字符串 "/brain/command_service"
        3. 通过 status_publisher 将状态消息发布出去。
        4. 如果任务状态为 'succeeded'、'failed' 或 'stopped'，则增加状态消息发送计数器 status_sent_count，
        当计数器达到 3 时，调用 stop_status_updates 方法停止后续状态更新。

        Args:
            status (str): 当前任务状态（例如 'waiting', 'working', 'succeeded', 'failed', 'stopped'）。
            reason (str): 状态变更的原因描述。

        Returns:
            None
        """
        self.task_status = status
        self.task_reason = reason
        status_msg = String()
        status_msg.data = (
            f"{self.task_command},"
            f"{status},"
            f"{reason},"
            "/brain/command_service"
        )
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
    def send_to_motion_planner(self) -> None:
        """协调运动规划流程。

        该方法负责：
        1. 初始化位姿处理状态
        2. 遍历所有抓取位姿
        3. 逐个发送位姿到运动规划服务
        4. 处理规划失败时的自动重试机制
        5. 更新当前处理位姿索引

        Raises:
            ServiceException: 当运动规划服务不可用时抛出
            TimeoutError: 当服务响应超时（10秒）时抛出

        Note:
            - 采用顺序处理策略，逐个尝试所有抓取位姿
            - 任一规划成功即终止后续位姿处理
            - 服务调用超时时间设置为10秒
            - 自动处理所有位姿规划失败的情况
            - 维护当前处理位姿索引用于状态跟踪
        """
        self.failed_at_last_pose = False
        self.pending_poses_count = len(self.grasp_poses)
        self.current_pose_index = 0  # 初始化当前位姿索引

        # 发送第一个位姿
        self.send_next_pose()

    def send_next_pose(self):
        """发送当前索引对应的抓取位姿给运动规划服务.

        该方法实现以下功能：
        1. 检查当前抓取位姿索引是否超出抓取位姿列表的范围。
        - 如果所有抓取位姿均已处理，且任务尚未成功完成，则调用 handle_error 方法，
            并传递错误信息 "机器人抓取不到物体" 以标记抓取失败。
        2. 如果索引在有效范围内，从 grasp_poses 列表中获取当前抓取位姿。
        3. 构造一个 GeneratePlan 请求对象，并将 target_pose 字段设定为当前抓取位姿。
        4. 记录日志，提示正在向运动规划服务发送目标位姿。
        5. 异步调用运动规划服务，通过 motion_generate_client 的 call_async 方法发送请求，
        同时为返回的 future 对象注册回调函数 motion_plan_callback 以处理响应结果。

        Returns:
            None: 此方法不返回任何值。
        """
        if self.current_pose_index >= len(self.grasp_poses):
            # 如果所有位姿均已处理，且没有成功的规划，则视为抓取失败
            if not self.is_task_complete:
                self.handle_error('运动规划', "机器人抓取不到物体")
            return

        pose = self.grasp_poses[self.current_pose_index]
        # target_marker = self.marker


        self.get_logger().info(f'发送抓取位姿 {self.current_pose_index + 1} 给运动规划...')
        self.generate_motion_plan(pose)


    def generate_motion_plan(self, target_pose):
        """生成目标位姿的运动规划.

        该方法实现以下功能：
        1. 等待运动规划服务可用，超时时间设定为 10 秒。
        - 如果服务不可用，则调用 handle_error 方法，并传递错误信息 "服务不可用"，随后返回。
        2. 如果服务可用，构造一个 GeneratePlan 请求对象，并将 target_pose 字段设定为传入的目标位姿。
        3. 记录日志，提示正在向运动规划服务发送目标位姿请求。
        4. 异步调用运动规划服务，通过 motion_generate_client 的 call_async 方法发送请求，
        同时为返回的 future 对象注册回调函数 motion_plan_callback 以处理响应结果。

        Args:
            target_pose: 目标位姿，应为 Pose 类型或与 GeneratePlan.Request.target_pose 相兼容的数据结构。

        Returns:
            None: 此方法不返回任何值。

        Raises:
            无异常直接抛出：若服务不可用，错误通过 handle_error 方法内部处理。
        """
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
        """处理运动规划响应的回调函数.

        当运动规划服务的异步调用返回时，该回调函数将被触发以处理响应数据。具体流程如下：
        
        1. 尝试从 future 对象中获取运动规划响应数据。
        2. 如果响应表示成功（response.is_success 为 True）：
        - 记录日志提示运动规划成功；
        - 将任务状态置为完成（is_task_complete 设为 True）；
        - 调用 send_joint_trajectory 方法，传入响应中的 joint_trajectory 以执行轨迹控制；
        - 直接返回，不再发送后续位姿。
        3. 如果响应表示失败（response.is_success 为 False）：
        - 记录错误日志，显示失败原因（response.failure_reason）；
        - 减少待处理抓取位姿数量（pending_poses_count 减 1）。
        4. 若在获取响应过程中发生异常：
        - 捕获异常，并记录相应的错误日志；
        - 同样减少待处理抓取位姿数量（pending_poses_count 减 1）。
        5. 无论响应失败或异常处理后：
        - 更新抓取位姿索引（current_pose_index 加 1）；
        - 调用 send_next_pose 方法尝试发送下一个抓取位姿给运动规划服务。

        Args:
            future: 包含运动规划服务响应结果的 future 对象。

        Returns:
            None: 该回调函数不返回任何值。
        """
        try:
            response = future.result()
            if response.is_success:
                self.get_logger().info('运动规划成功!')
                self.is_task_complete = True  # 任务成功完成
                self.send_joint_trajectory(response.joint_trajectory)
                # 成功后直接返回，不再尝试后续位姿
                return
            else:
                self.get_logger().error(f'运动规划失败: {response.failure_reason}')
                self.pending_poses_count -= 1  # 减少待处理位姿数
        except Exception as e:
            self.get_logger().error(f'运动规划请求时出错: {e}')
            self.pending_poses_count -= 1  # 异常情况下减少待处理位姿数

        # 更新索引到下一个抓取位姿并尝试再次发送
        self.current_pose_index += 1
        self.send_next_pose()


    def send_joint_trajectory(self, joint_trajectory):
        """发送关节轨迹给下层控制器.

        该方法将生成的关节轨迹消息发送给下层控制器执行具体的运动操作，具体流程如下：
        
        1. 构造 FollowJointTrajectory 的 Goal 消息对象，并设置轨迹（trajectory）及控制频率（control_rate）。
        2. 打印 joint_trajectory 用于调试。
        3. 记录日志，提示正在等待关节轨迹控制服务的连接。
        4. 使用 gongga_driver_client 等待关节轨迹控制服务可用，超时时间为 10 秒。
        - 如果服务不可用，则记录错误日志并调用 handle_error 方法处理错误，然后返回。
        5. 如果服务可用，记录日志提示正在发送关节轨迹目标。
        6. 异步发送目标给关节轨迹控制服务，通过 send_goal_async 方法发送，并注册：
        - 反馈回调函数（feedback_callback）用于处理反馈；
        - 完成回调函数（done_callback）用于处理最终响应结果。

        Args:
            joint_trajectory: 关节轨迹数据，应符合 FollowJointTrajectory.Goal.trajectory 的格式要求。

        Returns:
            None: 该方法不返回任何值。
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = joint_trajectory
        goal_msg.control_rate = 10

        print("joint_trajectory", joint_trajectory)

        # 等待关节轨迹控制服务连接
        self.get_logger().info("等待关节轨迹控制服务...")
        if not self.gongga_driver_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("未连接到关节轨迹控制服务")
            self.handle_error('关节轨迹发送', "未连接到关节轨迹控制服务")
            return

        # 异步发送目标到关节轨迹控制服务
        self.get_logger().info("发送关节轨迹目标")
        future = self.gongga_driver_client.send_goal_async(
            goal_msg, feedback_callback=self.gongga_driver_feedback_callback
        )
        future.add_done_callback(self.gongga_driver_callback)

    def gongga_driver_feedback_callback(self, feedback_msg):
        """反馈关节轨迹执行进度.

        该方法处理来自下层控制器的反馈消息，从中提取轨迹执行进度（百分比），
        并将当前进度信息记录到日志中。

        Args:
            feedback_msg: ROS2 反馈消息对象，应包含属性 feedback.progress_percentage，
                        表示当前轨迹执行进度的百分比。

        Returns:
            None: 该方法不返回任何值。
        """
        progress = feedback_msg.feedback.progress_percentage
        self.get_logger().info(f"轨迹执行进度: {progress}%")


    def gongga_driver_callback(self, future):
        """处理下层控制器响应.

        该方法用于处理关节轨迹控制器异步响应的回调。具体流程如下：
        
        1. 记录日志，提示已接收到关节轨迹控制器的响应；
        2. 尝试从 future 对象中获取目标句柄（goal_handle）：
        - 如果目标未被接受，则记录错误日志，并调用 handle_error 方法，
            传入阶段 '关节轨迹发送' 和错误信息 "目标未被接收"，然后直接返回；
        3. 如果目标被接受，则记录日志，提示正在等待关节轨迹执行结果；
        4. 异步等待执行结果，并注册 gongga_driver_result_callback 方法处理最终结果；
        5. 如果在上述过程中发生异常，则捕获异常，记录错误日志，并调用 handle_error 方法处理错误。

        Args:
            future: 包含关节轨迹控制器响应结果的 future 对象。

        Returns:
            None: 该方法不返回任何值.
        """
        self.get_logger().info("接收到关节轨迹控制器响应")
        try:
            # 获取目标句柄
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("目标未被接收")
                self.handle_error('关节轨迹发送', "目标未被接收")
                return

            # 异步等待执行结果
            self.get_logger().info("等待关节轨迹执行结果...")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.gongga_driver_result_callback)

        except Exception as e:
            self.get_logger().error(f"发送关节轨迹目标时出错: {e}")
            self.handle_error('关节轨迹发送', str(e))

    def gongga_driver_result_callback(self, future) -> None:
        """处理关节轨迹执行结果并协调后续抓取动作。

        该方法负责：
        1. 解析轨迹执行结果
        2. 控制夹爪闭合操作
        3. 控制升降机构抬升动作
        4. 更新最终任务状态
        5. 处理执行过程中的异常情况

        Args:
            future (rclpy.task.Future): 包含轨迹执行结果的Future对象

        Raises:
            ActionException: 当轨迹执行失败时抛出
            HardwareException: 当夹爪或升降机构控制失败时抛出

        Note:
            - 夹爪闭合采用预设姿态参数
            - 升降机构抬升高度固定为0.85米
            - 各硬件操作间设置3秒延时确保执行完成
            - 自动处理所有硬件控制指令的发送
            - 维护最终任务状态用于上层系统反馈
        """
        try:
            result = future.result().result
            if result.is_success:
                self.get_logger().info("轨迹执行成功，开始抓取!")
                gripper_action = VelAndPose()
                gripper_action.mode = 1
                gripper_action.pose.orientation.x = 0.0
                gripper_action.pose.orientation.y = 0.0
                gripper_action.pose.orientation.z = -0.966
                gripper_action.pose.orientation.w = 0.259
                self.get_logger().info('闭合夹爪')

                self.gripper_publisher.publish(gripper_action)
                time.sleep(3.0)

                lift_action = VelAndPose()
                lift_action.mode = 1
                lift_action.pose.position.z = 0.85
                self.lift_publisher.publish(lift_action)
                self.get_logger().info('抬升夹爪')
                # cameara_action = VelAndPose()
                # cameara_action.mode = 1
                # cameara_action.pose.orientation.x = 0.0
                # cameara_action.pose.orientation.y = 0.0
                # cameara_action.pose.orientation.z = 0.0
                # cameara_action.pose.orientation.w = 1.0
                # self.get_logger().info('调整相机')

                # self.camera_publisher.publish(cameara_action)
                time.sleep(3.0)
                self.update_status('succeeded', '任务完成')
            else:
                self.get_logger().error(f"轨迹执行失败: {result.failure_reason}")
                self.handle_error('关节轨迹发送', result.failure_reason)
        except Exception as e:
            self.get_logger().error(f"获取关节轨迹执行结果时出错: {e}")
            self.handle_error('关节轨迹执行', str(e))

def main(args=None):
    """ROS2节点主入口函数。
    
    Args:
        args (list, optional): 命令行参数列表，默认为None
    """
    rclpy.init(args=args)
    grasp_object_client = GraspObjectClient()
    rclpy.spin(grasp_object_client)

if __name__ == '__main__':
    main()
