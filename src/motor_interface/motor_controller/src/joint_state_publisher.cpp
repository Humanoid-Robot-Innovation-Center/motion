#include "joint_state_publisher.hpp"

JointStatePublisher::JointStatePublisher() 
    : Node("joint_state_publisher") 
{
    std::string stepper_device_port = " ";
    this->declare_parameter<std::string>("stepper_device_port");
    this->get_parameter("stepper_device_port", stepper_device_port);

    m_base_controller = std::make_shared<MotorController>(stepper_device_port, 115200, 8, 1, 21);
    m_lift_controller = std::make_shared<MotorController>(stepper_device_port, 115200, 8, 1, 23);
    m_arm_controller = std::make_shared<MotorController>(stepper_device_port, 115200, 8, 1, 24);

    std::string head_device_port = " ";
    std::string wrist_device_port = " ";
    this->declare_parameter<std::string>("head_device_port");
    this->declare_parameter<std::string>("wrist_device_port");
    this->declare_parameter<int>("id_wrist_roll");
    this->declare_parameter<int>("id_wrist_pitch");
    this->declare_parameter<int>("id_wrist_yaw");
    this->declare_parameter<int>("id_gripper_pull");
    this->declare_parameter<int>("id_head_pitch");
    this->declare_parameter<int>("id_head_yaw");
    this->declare_parameter<int>("id_camera_pitch");
    this->declare_parameter<int>("id_camera_yaw");
    this->get_parameter("head_device_port", head_device_port);
    this->get_parameter("wrist_device_port", wrist_device_port);
    this->get_parameter("id_wrist_roll", m_id_wrist_roll);
    this->get_parameter("id_wrist_pitch", m_id_wrist_pitch);
    this->get_parameter("id_wrist_yaw", m_id_wrist_yaw);
    this->get_parameter("id_gripper_pull", m_id_gripper_pull);
    this->get_parameter("id_head_pitch", m_id_head_pitch);
    this->get_parameter("id_head_yaw", m_id_head_yaw);
    this->get_parameter("id_camera_pitch", m_id_head_camera_pitch);
    this->get_parameter("id_camera_yaw", m_id_head_camera_yaw);

    m_wrist_driver = std::make_shared<SMS_STS>();
    m_wrist_driver->begin(1000000, wrist_device_port.c_str());
    m_head_driver = std::make_shared<SMS_STS>();
    m_head_driver->begin(1000000, head_device_port.c_str());

    joint_state_service_ = this->create_service<motor_interface::srv::GetJointState>(
        "get_joint_state", std::bind(&JointStatePublisher::handle_get_joint_state, this, std::placeholders::_1, std::placeholders::_2));

    // 创建定时器，每0.1秒发布一次
    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(100),
    //     [this]() { publish_joint_state(); }
    // );


}

void JointStatePublisher::handle_get_joint_state(const std::shared_ptr<motor_interface::srv::GetJointState::Request> request,
    std::shared_ptr<motor_interface::srv::GetJointState::Response> response)
{
    sensor_msgs::msg::JointState joint_state;
    // joint_state.name = {"jointyl", "jointzl", "jointshou", "jointss3", "jointsw1", "jointsw2", "jointsw4", "jointsw3", "jointtou1", "jointtou2"};

    // joint_state.name.resize(10);
    // joint_state.position.resize(10);
    joint_state.velocity.resize(9, 0.0);

    joint_state.name.push_back("joint_right_wheel");
    joint_state.position.push_back(0.0);

    joint_state.name.push_back("joint_left_wheel");
    joint_state.position.push_back(0.0);

    JointState lift_state;
    liftCallback(lift_state);
    joint_state.name.push_back(lift_state.name);
    joint_state.position.push_back(lift_state.position);

    JointState arm_state;
    armCallback(arm_state);
    joint_state.name.push_back(arm_state.name);
    joint_state.position.push_back(arm_state.position);

    JointState wrist_roll_state;
    wristRollCallback(wrist_roll_state);
    joint_state.name.push_back(wrist_roll_state.name);
    joint_state.position.push_back(wrist_roll_state.position);

    JointState wrist_pitch_state;
    wristPitchCallback(wrist_pitch_state);
    joint_state.name.push_back(wrist_pitch_state.name);
    joint_state.position.push_back(wrist_pitch_state.position);

    JointState wrist_yaw_state;
    wristYawCallback(wrist_yaw_state);
    joint_state.name.push_back(wrist_yaw_state.name);
    joint_state.position.push_back(wrist_yaw_state.position);

    // JointState gripper_state;
    // gripperPullCallback(gripper_state);
    // joint_state.name.push_back(gripper_state.name);
    // joint_state.position.push_back(gripper_state.position);

    JointState head_camera_pitch_state;
    headYawCallback(head_camera_pitch_state);
    joint_state.name.push_back(head_camera_pitch_state.name);
    joint_state.position.push_back(head_camera_pitch_state.position);

    JointState head_camera_yaw_state;
    headPitchCallback(head_camera_yaw_state);
    joint_state.name.push_back(head_camera_yaw_state.name);
    joint_state.position.push_back(head_camera_yaw_state.position);

    joint_state.header.stamp = this->now();

    response->joint_state = joint_state;

    // joint_state_publisher_->publish(joint_state);
    // RCLCPP_INFO(this->get_logger(), "Published joint states: [%f, %f, %f]", 
    //             joint_state.position[0], joint_state.position[1], joint_state.position[2]);
}


void JointStatePublisher::headPitchCallback(JointState& state)
{
    state.name = "joint_head_tilt";
    int step = m_head_driver->ReadPos(m_id_head_camera_pitch);
    state.position = (2048 - step) * 0.08789 / (180 / 3.14);
}

void JointStatePublisher::headYawCallback(JointState& state)
{
    state.name = "joint_head_pan";
    int step = m_head_driver->ReadPos(m_id_head_camera_yaw);
    state.position = (2048 - step) * 0.08789 / (180 / 3.14);
}

void JointStatePublisher::wristRollCallback(JointState& state)
{
    state.name = "joint_roll";
    int step = m_wrist_driver->ReadPos(m_id_wrist_roll);
    state.position = (2048 - step) * 0.08789 / (180 / 3.14);
}

void JointStatePublisher::wristPitchCallback(JointState& state)
{
    state.name = "joint_pitch";
    int step = m_wrist_driver->ReadPos(m_id_wrist_pitch);
    state.position = (step - 2048) * 0.08789 / (180 / 3.14);
}

void JointStatePublisher::wristYawCallback(JointState& state)
{
    state.name = "joint_yaw";
    int step = m_wrist_driver->ReadPos(m_id_wrist_yaw);
    state.position = (step - 2048) * 0.08789 / (180 / 3.14);
}

void JointStatePublisher::gripperPullCallback(JointState& state)
{
    state.name = "jointsw3";
    int step = m_wrist_driver->ReadPos(m_id_gripper_pull);
    state.position = (2048 - step) * 0.08789 / (180 / 3.14);
}

void JointStatePublisher::liftCallback(JointState& state)
{
    state.name = "joint_lift";
    uint16_t pose_l = 0;
    uint16_t pose_h = 0;
    // usleep(20000);
    m_lift_controller->readRegister(0x09, pose_l);
    // usleep(20000);
    m_lift_controller->readRegister(0x08, pose_h);

    int32_t pulse = ((uint32_t)pose_h << 16) | (uint32_t)pose_l;
    std::cout << "lift pose_l: " << pose_l << "; pose_h: " << pose_h << "; pulse: " << pulse << std::endl;
    // 零位处的jointState为绝对0值
    if ((double)(abs(pulse)) / 1000.0 * 0.018698318 < 0.005)
    {
        state.position = 0.0;
	std::cout << "11" << std::endl;
    }
    else
    {
        state.position = (double)pulse / 1000.0 * 0.018698318;
    }
}

void JointStatePublisher::armCallback(JointState& state)
{
    state.name = "joint_arm_L0";
    uint16_t pose_l = 0;
    uint16_t pose_h = 0;
    m_arm_controller->readRegister(0x09, pose_l);
    m_arm_controller->readRegister(0x08, pose_h);

    int32_t pulse = ((uint32_t)pose_h << 16) | (uint32_t)pose_l;
    std::cout << "arm pose_l: " << pose_l << "; pose_h: " << pose_h << "; pulse: " << pulse << std::endl;
    // 零位处的jointState为绝对0值
    if ((double)(abs(pulse)) / 1000.0 * 0.04008524 < 0.005)
    {
        state.position = 0.0;
    }
    else
    {
        state.position = (double)(abs(pulse)) / 1000.0 * 0.04008524;
    }
}

double JointStatePublisher::get_motor_feedback(size_t joint_index)
{
    // 示例：返回模拟的电机反馈，替换为实际获取电机数据的代码
    return joint_index * 0.1; // 例如，返回0.0, 0.1, 0.2等
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
