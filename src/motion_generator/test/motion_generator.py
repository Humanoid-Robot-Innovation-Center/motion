from rclpy.node import Node
from gongga_interface.srv import GeneratePlan
from sensor_msgs.msg import JointState as JS
from motor_interface.srv import GetJointState


class MotionGenerator(Node):
    def __init__(self):
        super().__init__('motion_generator')

        # todo:由于硬件限制，目前放弃该方式获取joint_states而采用service的方式获取
        # self.joint_state_subscription = self.create_subscription(
        #     JS,
        #     '/joint_states',
        #     self.joint_states_callback,
        #     10
        # )
        # self.joint_state_subscription
        self.client = self.create_client(GetJointState, '/get_joint_state')
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get Joint State Service not available, waiting again...')
        self.joint_states_req = GetJointState.Request()

        self.ee_translation_goal = None
        self.ee_orientation_teleop_goal = None
        self.pose_metric = None

        self.cmd_plan = None
        self.is_generation_success = False
        self.generate_plan_srv = self.create_service(GeneratePlan, '/gongga_planner/motion_generator', self.generate_plan_callback)

        self.joint_names = None
        self.joint_states = None
        self.joint_positions = None
        self.joint_velocities = None

        self.track_width = 0.3492  # unit:m
        self.wheel_radius = 0.05

        self.failure_reason = "unknown"

    def update_joint_states(self):
        raise NotImplementedError()

    def generate_plan_callback(self, request, response):
        raise NotImplementedError()

    def get_generation_status(self) -> bool:
        raise NotImplementedError()

    def get_failure_reason(self) -> str:
        raise NotImplementedError()

    def attach_object_to_kinematics(self):
        pass

    def detach_object_from_kinematics(self):
        pass

    def joint_states_callback(self, msg):
        raise NotImplementedError()

    def get_response_joint_trajectory(self):
        joint_trajectory = JS()
        for i in range(len(self.cmd_plan.position)):
            joint_trajectory.name = self.cmd_plan[i].joint_names
            for pos in self.cmd_plan[i].position.cpu().numpy().tolist():
                joint_trajectory.position.append(pos)
            for vel in self.cmd_plan[i].velocity.cpu().numpy().tolist():
                joint_trajectory.velocity.append(vel)
        return joint_trajectory

