#include "lichuan_stepper_motor/driver_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

#define AXIS1_DI_FUNC_ADDRESS 0x0022                // 轴1 DI1功能选择的地址(PA_029)
#define AXIS2_DI_FUNC_ADDRESS 0x0122                // 轴2 DI1功能选择的地址(PA_068)
#define AXIS1_DI_LOGIC_ADDRESS 0x001F               // 轴1 DI1输入逻辑的地址(PA_026)
#define AXIS2_DI_LOGIC_ADDRESS 0x011F               // 轴2 DI1输入逻辑的地址(PA_065)
#define DI_POSE_MODE 0x20                           // DI功能位置模式(0x20 - 32d)
#define DI_SPEED_MODE 0x23                          // DI功能速度模式(0x23 - 35d)
#define DI_LOGIC_0 0x00                             // DI逻辑状态0
#define DI_LOGIC_1 0x01                             // DI逻辑状态1

#define AXIS1_JOG_SPEED_ADDRESS 2009                // 轴1 JOG速度的地址(PA_108)
#define AXIS2_JOG_SPEED_ADDRESS 2409                // 轴2 JOG速度的地址(PA_193)

#define AXIS1_CONTROL_ADDRESS 0x2002                // 轴1 运行控制字的地址(PA_101)
#define AXIS2_CONTROL_ADDRESS 0x2402                // 轴2 运行控制字的地址(PA_186)
#define CONTROL_START 0x10                          // 控制字-启动
#define CONTROL_STOP 0x00                           // 控制字-停止
#define AXIS1_POSE_MODE_SPEED_ADDRESS 0x2203        // 轴1 位置模式下的速度
#define AXIS2_POSE_MODE_SPEED_ADDRESS 0x2603        // 轴2 位置模式下的速度

int32_t calComplement(int32_t num)
{
    return ~num + 1;
}

DriverNode::DriverNode()
: Node("driver_node")
{
    m_last_time = this->get_clock()->now();

    // 解析并获取参数
    this->declare_parameter<std::string>("device_port");
    this->declare_parameter<int>("baud");
    this->declare_parameter<int>("data_bit");
    this->declare_parameter<int>("stop_bit");
    this->declare_parameter<int>("base_slave_id");
    this->declare_parameter<int>("lift_slave_id");
    this->declare_parameter<int>("arm_slave_id");
    this->declare_parameter<double>("lift_reduction_ratio");
    this->declare_parameter<double>("base_reduction_ratio");
    this->declare_parameter<double>("arm_reduction_ratio");
    this->declare_parameter<double>("wheel_base");
    this->declare_parameter<double>("wheel_radius");
    this->get_parameter("device_port", m_device_port);
    this->get_parameter("baud", m_baud);
    this->get_parameter("data_bit", m_data_bit);
    this->get_parameter("stop_bit", m_stop_bit);
    this->get_parameter("base_slave_id", m_base_slave_id);
    this->get_parameter("lift_slave_id", m_lift_slave_id);
    this->get_parameter("arm_slave_id", m_arm_slave_id);
    this->get_parameter("lift_reduction_ratio", m_lift_reduction_ratio);
    this->get_parameter("base_reduction_ratio", m_base_reduction_ratio);
    this->get_parameter("arm_reduction_ratio", m_arm_reduction_ratio);
    this->get_parameter("wheel_base", m_wheel_base);
    this->get_parameter("wheel_radius", m_wheel_radius);
    RCLCPP_INFO(this->get_logger(), "m_device_port: %s, m_baud: %d, m_data_bit: %d, m_stop_bit: %d",
        m_device_port.c_str(), m_baud, m_data_bit, m_stop_bit);
    RCLCPP_INFO(this->get_logger(), "m_base_slave_id: %d, m_lift_slave_id: %d, m_arm_slave_id: %d",
        m_base_slave_id, m_lift_slave_id, m_arm_slave_id);
    RCLCPP_INFO(this->get_logger(), "m_base_reduction_ratio: %f, m_lift_reduction_ratio: %f, m_arm_reduction_ratio: %f",
        m_base_reduction_ratio, m_lift_reduction_ratio, m_arm_reduction_ratio);

    // 初始化
    m_base_controller = std::make_shared<MotorController>(m_device_port, m_baud, m_data_bit, m_stop_bit, m_base_slave_id);
    m_lift_controller = std::make_shared<MotorController>(m_device_port, m_baud, m_data_bit, m_stop_bit, m_lift_slave_id);
    m_arm_controller = std::make_shared<MotorController>(m_device_port, m_baud, m_data_bit, m_stop_bit, m_arm_slave_id);

    sleep(2);
    setZero();
    sleep(5);

    // 底盘订阅速度和位姿
    m_base_vel_pose_sub = this->create_subscription<motor_interface::msg::VelAndPose>(
        "cmd_vel_pose",
        10,
        std::bind(&DriverNode::handBaseVelAndPose, this, std::placeholders::_1)
    );

    // 底盘订阅速度
    m_base_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        std::bind(&DriverNode::handBaseVel, this, std::placeholders::_1)
    );

    // 升降杆订阅速度和位姿
    m_lift_sub = this->create_subscription<motor_interface::msg::VelAndPose>(
        "cmd_lift",
        10,
        std::bind(&DriverNode::handLift, this, std::placeholders::_1)
    );

    // 伸缩杆订阅速度和位姿
    m_arm_sub = this->create_subscription<motor_interface::msg::VelAndPose>(
        "cmd_arm",
        10,
        std::bind(&DriverNode::handArm, this, std::placeholders::_1)
    );

    // m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    // m_odom_thread = std::thread(&DriverNode::publishOdom, this);
}

DriverNode::~DriverNode()
{
    if (m_odom_thread.joinable()) 
    {
        m_odom_thread.join();  // 等待线程结束
    }
}

void DriverNode::handBaseVelAndPose(const motor_interface::msg::VelAndPose::SharedPtr msg)
{
    // 解析 mode 和速度
    int mode = msg->mode;

    /* 速度模式 */
    if (mode == 0)
    {
        double linear_velocity = msg->vel.linear.x;
        double angular_velocity = msg->vel.angular.z;

        // 左右轮线速度
        double l_velocity = linear_velocity - (getWheelBase() / 2.0) * angular_velocity;
        double r_velocity = linear_velocity + (getWheelBase() / 2.0) * angular_velocity;

        // 左右电机转速
        // int l_rpm = std::round(l_velocity / (2 * 3.14 * getWheelRadius()) * 60 * m_base_reduction_ratio);
        // int r_rpm = std::round(r_velocity / (2 * 3.14 * getWheelRadius()) * 60 * m_base_reduction_ratio);
        int l_rpm = std::round(l_velocity / 0.081593333 * 60); // 传动比
        int r_rpm = std::round(r_velocity / 0.081593333 * 60);

        // 设置速度
        m_base_controller->setSpeed(l_rpm, r_rpm);

        RCLCPP_INFO(this->get_logger(), "l_velocity: %f, r_velocity: %f, l_rpm: %d, r_rpm: %d", l_velocity, r_velocity, l_rpm, r_rpm);
    }

    /* 位置模式 */
    if (mode == 1)
    {
        tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        m.getRPY(roll, pitch, yaw);

        // tf2::Quaternion last_q(m_last_x, m_last_y, m_last_z, m_last_w);
        // tf2::Matrix3x3 last_m(last_q);
        // double last_roll = 0.0;
        // double last_pitch = 0.0;
        // double last_yaw = 0.0;
        // last_m.getRPY(last_roll, last_pitch, last_yaw);

        // double delta_d = sqrt(pow(m_last_pose_x - msg->pose.position.x, 2) + pow(m_last_pose_y - msg->pose.position.y, 2));
        // double delta_theta = last_yaw - yaw;

        double delta_d = sqrt(pow(msg->pose.position.x, 2) + pow(msg->pose.position.y, 2));
        double delta_theta = yaw;
        if (msg->pose.position.x < 0)
        {
            delta_d = -delta_d;
        }
        RCLCPP_INFO(this->get_logger(), "delta_d: %f, delta_theta: %f", delta_d, delta_theta);

        double l_d = delta_d - getWheelBase() / 2.0 * delta_theta;
        double r_d = delta_d + getWheelBase() / 2.0 * delta_theta;

        RCLCPP_INFO(this->get_logger(), "l_d: %f, r_d: %f", l_d, r_d);

        // double l_r = l_d / (2 * 3.14 * getWheelRadius());
        // double r_r = r_d / (2 * 3.14 * getWheelRadius());
        double l_r = l_d / 0.081593333;
        double r_r = r_d / 0.081593333;

        RCLCPP_INFO(this->get_logger(), "l_r: %f, r_r: %f", l_r, r_r);

        m_base_controller->setPosition(l_r, r_r);

        // m_last_pose_x = msg->pose.position.x;
        // m_last_pose_y = msg->pose.position.y;
        // m_last_x      = msg->pose.orientation.x;
        // m_last_y      = msg->pose.orientation.y;
        // m_last_z      = msg->pose.orientation.z;
        // m_last_w      = msg->pose.orientation.w;
    }
}

void DriverNode::handBaseVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // 解析速度
    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;
    if (fabs(m_last_vel - linear_velocity) > 0.0)
    {
        for (size_t i = 1; i <= 5; i++)
        {
            double new_linear_velocity = m_last_vel + (linear_velocity - m_last_vel) * (double)(i/5.0);

            // 左右轮线速度
            double l_velocity = new_linear_velocity - (getWheelBase() / 2.0) * angular_velocity;
            double r_velocity = new_linear_velocity + (getWheelBase() / 2.0) * angular_velocity;

            // 左右电机转速
            // int l_rpm = std::round(l_velocity / (2 * 3.14 * getWheelRadius()) * 60 * m_base_reduction_ratio);
            // int r_rpm = std::round(r_velocity / (2 * 3.14 * getWheelRadius()) * 60 * m_base_reduction_ratio);
            int l_rpm = std::round(l_velocity / 0.0942 * 60);
            int r_rpm = std::round(r_velocity / 0.0942 * 60);
	    // l_rpm = 0;
	    // r_rpm = 0;

            // 设置速度
            m_base_controller->setSpeed(l_rpm, r_rpm);

            RCLCPP_INFO(this->get_logger(), "l_velocity: %f, r_velocity: %f, l_rpm: %d, r_rpm: %d", l_velocity, r_velocity, l_rpm, r_rpm);
            usleep(20000);
        }
        m_last_vel = linear_velocity;
    }
    else
    {
    // 解析速度
    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;

    // 左右轮线速度
    double l_velocity = linear_velocity - (getWheelBase() / 2.0) * angular_velocity;
    double r_velocity = linear_velocity + (getWheelBase() / 2.0) * angular_velocity;

    // 左右电机转速
    // int l_rpm = std::round(l_velocity / (2 * 3.14 * getWheelRadius()) * 60 * m_base_reduction_ratio);
    // int r_rpm = std::round(r_velocity / (2 * 3.14 * getWheelRadius()) * 60 * m_base_reduction_ratio);
    int l_rpm = std::round(l_velocity / 0.0942 * 60);
    int r_rpm = std::round(r_velocity / 0.0942 * 60);
	    // l_rpm = 0;
	    // r_rpm = 0;

    // 设置速度
    m_base_controller->setSpeed(l_rpm, r_rpm);

    RCLCPP_INFO(this->get_logger(), "l_velocity: %f, r_velocity: %f, l_rpm: %d, r_rpm: %d", l_velocity, r_velocity, l_rpm, r_rpm);
    }
}


void DriverNode::handLift(const motor_interface::msg::VelAndPose::SharedPtr msg)
{
    int mode = msg->mode;

    if (mode == 0)
    {
        double linear_velocity = msg->vel.linear.z;

        int rpm = std::round(linear_velocity / 0.018698318 * 60);

	if(rpm != 0)
	{
            // 设置速度
            m_lift_controller->writeSingleRegister(0x0011, 0x0D);
            m_lift_controller->writeSingleRegister(0xB0, rpm>0 ? rpm : calComplement(abs(rpm)));
            m_lift_controller->writeSingleRegister(0x0010, 0x01);
	}
	else
	{
            m_lift_controller->writeSingleRegister(0x0010, 0x00);
	}

        RCLCPP_INFO(this->get_logger(), "cmd_lift: linear_velocity: %f, rpm: %d", linear_velocity, rpm);
        m_haveSetVelMode = true;
    }
    else if (mode == 1)
    {
        m_lift_times++;
        RCLCPP_INFO(this->get_logger(), "cmd_lift_times: %d", m_lift_times);
        // if (m_haveSetVelMode)
        // {
        //     setZero();
        //     m_lift_controller->writeSingleRegister(0x0010, 0x00);
        //     m_haveSetVelMode = false;
        //     sleep(5);
        // }
        // 接收位置为正时, 立柱升; 为负时, 立柱降
        float z = msg->pose.position.z;
        int pulse = z / 0.018698318 * 1000;
        if (z < 0.0)
        {
            RCLCPP_INFO(this->get_logger(), "<0:  cmd_lift: z: %f, pulse: %d, L: %x, H: %x", z, pulse, (calComplement(pulse) & 0xFFFF), ((calComplement(pulse) >> 16) & 0xFFFF));
        }
        else if(z > 0.0)
        {
            RCLCPP_INFO(this->get_logger(), ">0:  cmd_lift: z: %f, pulse: %d, L: %x, H: %x", z, pulse, ((pulse) & 0xFFFF), ((pulse >> 16) & 0xFFFF));
            std::vector<uint16_t> data = {0x0000, 0x0064, 0x0064, 0x0064, ((pulse >> 16) & 0xFFFF), ((pulse) & 0xFFFF)};
            m_lift_controller->writeMultipleRegisters(0x0033, data);
            m_lift_controller->writeSingleRegister(0x4E, 0x0007);
        }
    }
    else if (mode == 2)
    {
        // 接收位置为正时, 立柱升; 为负时, 立柱降
        float z = msg->pose.position.z;
        int pulse = z / 0.018698318 * 1000;
        if (z < 0.0)
        {
             
        }
        else if(z > 0.0)
        {
            RCLCPP_INFO(this->get_logger(), ">0:  cmd_lift: z: %f, pulse: %d, L: %x, H: %x", z, pulse, ((pulse) & 0xFFFF), ((pulse >> 16) & 0xFFFF));
            std::vector<uint16_t> data = {0x0000, 0x0064, 0x0064, 0x0064, ((pulse >> 16) & 0xFFFF), ((pulse) & 0xFFFF)};
            m_lift_controller->writeMultipleRegisters(0x0033, data);
            m_lift_controller->writeSingleRegister(0x4E, 0x0007);
        }
    }

}


void DriverNode::handArm(const motor_interface::msg::VelAndPose::SharedPtr msg)
{
    int mode = msg->mode;

    if (mode == 0)
    {
        double linear_velocity = -msg->vel.linear.x;
        int rpm = std::round(linear_velocity / 0.04008524 * 60);

	if (rpm != 0)
	{
        // 设置速度
        m_arm_controller->writeSingleRegister(0x0011, 0x0D);
        m_arm_controller->writeSingleRegister(0xB0, rpm>0 ? rpm : calComplement(abs(rpm)));
        m_arm_controller->writeSingleRegister(0x0010, 0x01);
	}
	else
	{
            m_arm_controller->writeSingleRegister(0x0010, 0x00);
	}

        RCLCPP_INFO(this->get_logger(), "cmd_arm: linear_velocity: %f, rpm: %d", linear_velocity, rpm);
    }

    if (mode == 1)
    {
        RCLCPP_INFO(this->get_logger(), "cmd_arm: vel_x: %f, pos_x: %f", msg->vel.linear.x, msg->pose.position.x);
        uint16_t vel = msg->vel.linear.x / 0.0308348 * 60;

        float origin_x = msg->pose.position.x;
        RCLCPP_INFO(this->get_logger(), "origin_x: %f, m_last_arm_pose: %f", origin_x, m_last_arm_pose);
        if ((origin_x - m_last_arm_pose > 0 && !m_is_arm_pose_increase) || (origin_x - m_last_arm_pose < 0 && m_is_arm_pose_increase))
        {
            // origin_x = (origin_x - m_last_arm_pose > 0) ? origin_x+0.015 : origin_x-0.015;
            m_delta_arm_pose = (origin_x - m_last_arm_pose > 0) ? m_delta_arm_pose+0.015 : m_delta_arm_pose-0.015;
                RCLCPP_INFO(this->get_logger(), "ori_x-m_last: %f; m_is: %d; m_del: %f", origin_x-m_last_arm_pose, m_is_arm_pose_increase, m_delta_arm_pose);
        }
        
        if (origin_x - m_last_arm_pose > 0)
            {
                RCLCPP_INFO(this->get_logger(), "111111");
                m_is_arm_pose_increase = true;
        }
        else if (origin_x - m_last_arm_pose < 0)
        {
                RCLCPP_INFO(this->get_logger(), "22222");
                m_is_arm_pose_increase = false;
        }
        m_last_arm_pose = msg->pose.position.x;
        
        RCLCPP_INFO(this->get_logger(), "origin_x: %f, m_last_arm_pose: %f", origin_x, m_last_arm_pose);
    float x = origin_x + m_delta_arm_pose;
        int pulse = x / 0.04008524 * 1000;
        RCLCPP_INFO(this->get_logger(), "cmd_arm: vel: %d, pulse: %d", vel, pulse);
        if (x < 0.0)
        {
            // m_arm_controller->writeSingleRegister(0x2402, 0x00);
            // m_arm_controller->writeSingleRegister(0x2601, (calComplement(abs(pulse)) & 0xFFFF));    // 相对位置运行
            // m_arm_controller->writeSingleRegister(0x2602, ((calComplement(abs(pulse)) >> 16) & 0xFFFF));    // 相对位置运行
            // m_arm_controller->writeSingleRegister(0x2600, 0x80);    // 相对位置运行
            // m_arm_controller->writeSingleRegister(0x2603, abs(vel));    // 转速100r/min
            // m_arm_controller->writeSingleRegister(0x2402, 0x10);    // 启动
        }
        else if (x > 0.0)
        {
            // m_arm_controller->writeSingleRegister(0x2402, 0x00);
            // m_arm_controller->writeSingleRegister(0x2601, ((pulse) & 0xFFFF));    // 相对位置运行
            // m_arm_controller->writeSingleRegister(0x2602, ((pulse >> 16) & 0xFFFF));    // 相对位置运行
            // m_arm_controller->writeSingleRegister(0x2600, 0x80);    // 相对位置运行
            // m_arm_controller->writeSingleRegister(0x2603, vel);    // 转速100r/min
            // m_arm_controller->writeSingleRegister(0x2402, 0x10);    // 启动

            RCLCPP_INFO(this->get_logger(), ">0:  cmd_arm: x: %f, pulse: %d, L: %x, H: %x", x, pulse, (calComplement(pulse) & 0xFFFF), ((calComplement(pulse) >> 16) & 0xFFFF));
            std::vector<uint16_t> data = {0x0000, 0x0064, 0x0064, 0x0064, ((calComplement(pulse) >> 16) & 0xFFFF), (calComplement(pulse) & 0xFFFF)};
            m_arm_controller->writeMultipleRegisters(0x0033, data);
            m_arm_controller->writeSingleRegister(0x4E, 0x0007);
        }
    }
}

void DriverNode::publishOdom()
{
    try
    {
        uint16_t l_r = 0;
        uint16_t r_r = 0;

        std::unique_lock<std::mutex> lock(m_controller_mutex);
        if (m_base_controller) 
        {
            m_base_controller->readRegister(0x0316, l_r);
            m_base_controller->readRegister(0x0317, r_r);
        }
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Base controller is not initialized.");
            return; // 如果控制器未初始化，直接返回
        }
        lock.unlock();

        // 左右轮线速度
        double l_v = l_r / 60 * 2 * 3.14 * getWheelRadius();
        double r_v = r_r / 60 * 2 * 3.14 * getWheelRadius();

        // 线速度、角速度
        double v = (l_v + r_v) / 2.0;
        double omega = getWheelRadius() * (r_v - l_v) / getWheelBase();

        auto current_time = this->get_clock()->now();
        auto dt = current_time - m_last_time;

        // 计算时间步长（单位为秒）
        double dt_seconds = dt.seconds();

        // 假设 v 和 omega 已经被获取
        // double delta_x = v * dt_seconds;
        double delta_x = v * dt_seconds * cos(m_current_theta);
        double delta_y = v * dt_seconds * sin(m_current_theta);
        double delta_theta = omega * dt_seconds;

        // 更新位置信息
        m_current_x += delta_x;
        m_current_y += delta_y;
        m_current_theta += delta_theta;

        m_last_time = current_time; // 更新最后时间

        nav_msgs::msg::Odometry odometry_msg;
        odometry_msg.header.stamp = current_time;
        odometry_msg.header.frame_id = "odom";
        odometry_msg.pose.pose.position.x = m_current_x;
        odometry_msg.pose.pose.position.y = m_current_y;
        odometry_msg.pose.pose.orientation.z = sin(m_current_theta / 2);
        odometry_msg.pose.pose.orientation.w = cos(m_current_theta / 2);
        m_odom_pub->publish(odometry_msg);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception in publishOdom: %s", e.what());
    }
    


}


void DriverNode::setZero()
{

    m_lift_controller->writeSingleRegister(0x1E, 0x05);

sleep(1);
    m_lift_controller->writeSingleRegister(0x4F, 0x0600);
sleep(1);
    m_lift_controller->writeSingleRegister(0x4F, 0x0500);
sleep(1);
    // 往上抬升一点
    std::vector<uint16_t> data = {0x0000, 0x0064, 0x0064, 0x0064, 0x0000, 0x07D0};
    m_lift_controller->writeMultipleRegisters(0x0033, data);
    m_lift_controller->writeSingleRegister(0x4E, 0x0001);
    // 往前伸一点
    // std::vector<uint16_t> data1 = {0x0000, 0x0064, 0x0064, 0x0064, 0xFFFF, 0xFC18};
    // m_arm_controller->writeMultipleRegisters(0x0033, data1);
    // m_arm_controller->writeSingleRegister(0x4E, 0x0001);

    sleep(2);
    
    // 升降回零
    m_lift_controller->writeSingleRegister(0x11, 0x01);
    m_lift_controller->writeSingleRegister(0x40, 0x1D);
    // m_lift_controller->writeSingleRegister(0x41, 0xC8);
    m_lift_controller->writeSingleRegister(0x4E, 0x10);

    // // 伸缩回零
    m_arm_controller->writeSingleRegister(0x11, 0x01);
    m_arm_controller->writeSingleRegister(0x40, 0x18);
    // m_arm_controller->writeSingleRegister(0x41, 0xC8);
    m_arm_controller->writeSingleRegister(0x4E, 0x10);
}
