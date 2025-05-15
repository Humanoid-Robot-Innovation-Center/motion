#include <rclcpp/rclcpp.hpp>
#include <motor_interface/msg/vel_and_pose.hpp>
#include <std_msgs/msg/float64.hpp>
#include "feetech_servo_motor/SMS_STS.h" // 包含SMS_STS头文件
#include <yaml-cpp/yaml.h>
#include <map>
#include <cstring>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>

// 头部yaw关节-ID:15(0x0F) 角度限制:1000-4095
// 头部pitch关节:ID:16(0x10) 角度限制:900-2400

// 灵巧手水平关节1-ID:11(0x0B)    角度限制:1000-3100
// 灵巧手俯仰关节2-ID:12(0x0C)    角度限制:2048-3000  
// 灵巧手旋转关节3-ID:13(0x0D)    角度限制:0-4095
// 灵巧手夹爪关节4-ID:14(0x0E)    角度限制:0-4095
// 头部水平关节-ID:15(0x0F)   角度限制:1000-4095
// 头部俯仰关节:ID:16(0x10)   角度限制:900-2400

#define ANGLE_RESOLUTION 0.08789

class MotorControllerNode : public rclcpp::Node
{
public:
    MotorControllerNode()
    : Node("motor_controller_node")
    {
        // loadMotorIDs();
        this->declare_parameter<std::string>("wrist_port");
        this->declare_parameter<std::string>("head_port");
        this->declare_parameter<int>("baud");
        this->declare_parameter<int>("id_wrist_roll");
        this->declare_parameter<int>("id_wrist_pitch");
        this->declare_parameter<int>("id_wrist_yaw");
        this->declare_parameter<int>("id_gripper_pull");
        this->declare_parameter<int>("id_head_pitch");
        this->declare_parameter<int>("id_head_yaw");
        this->declare_parameter<int>("id_camera_pitch");
        this->declare_parameter<int>("id_camera_yaw");

        std::string wrist_port = "";
        std::string head_port = "";
        int baud = 0;

        this->get_parameter("wrist_port", wrist_port);
        this->get_parameter("head_port", head_port);
        this->get_parameter("baud", baud);
        this->get_parameter("id_wrist_roll", m_id_wrist_roll);
        this->get_parameter("id_wrist_pitch", m_id_wrist_pitch);
        this->get_parameter("id_wrist_yaw", m_id_wrist_yaw);
        this->get_parameter("id_gripper_pull", m_id_gripper_pull);
        this->get_parameter("id_head_pitch", m_id_head_pitch);
        this->get_parameter("id_head_yaw", m_id_head_yaw);
        this->get_parameter("id_camera_pitch", m_id_camera_pitch);
        this->get_parameter("id_camera_yaw", m_id_camera_yaw);

        RCLCPP_INFO(this->get_logger(), "wrist_port: %s, head_port: %s, baud: %d", wrist_port.c_str(), head_port.c_str(), baud);
        RCLCPP_INFO(this->get_logger(), "id_wrist_roll: %d, id_wrist_pitch: %d, id_wrist_yaw: %d, id_gripper_pull: %d, id_head_pitch: %d, id_head_yaw: %d, id_camera_pitch: %d, id_camera_yaw: %d",
            m_id_wrist_roll, m_id_wrist_pitch, m_id_wrist_yaw, m_id_gripper_pull, m_id_head_pitch, m_id_head_yaw, m_id_camera_pitch, m_id_camera_yaw);

        wrist_roll_sub_ = this->create_subscription<motor_interface::msg::VelAndPose>(
            "wrist_roll", 10, std::bind(&MotorControllerNode::wristRollCallback, this, std::placeholders::_1));

        wrist_pitch_sub_ = this->create_subscription<motor_interface::msg::VelAndPose>(
            "wrist_pitch", 10, std::bind(&MotorControllerNode::wristPitchCallback, this, std::placeholders::_1));

        wrist_yaw_sub_ = this->create_subscription<motor_interface::msg::VelAndPose>(
            "wrist_yaw", 10, std::bind(&MotorControllerNode::wristYawCallback, this, std::placeholders::_1));

        gripper_pull_sub_ = this->create_subscription<motor_interface::msg::VelAndPose>(
            "gripper_pull", 10, std::bind(&MotorControllerNode::gripperPullCallback, this, std::placeholders::_1));

        head_pitch_sub_ = this->create_subscription<motor_interface::msg::VelAndPose>(
            "head_pitch", 10, std::bind(&MotorControllerNode::headPitchCallback, this, std::placeholders::_1));

        head_yaw_sub_ = this->create_subscription<motor_interface::msg::VelAndPose>(
            "head_yaw", 10, std::bind(&MotorControllerNode::headYawCallback, this, std::placeholders::_1));

        camera_pitch_sub_ = this->create_subscription<motor_interface::msg::VelAndPose>(
            "camera_pitch", 10, std::bind(&MotorControllerNode::cameraPitchCallback, this, std::placeholders::_1));

        camera_yaw_sub_ = this->create_subscription<motor_interface::msg::VelAndPose>(
            "camera_yaw", 10, std::bind(&MotorControllerNode::cameraYawCallback, this, std::placeholders::_1));

        m_wrist_driver = std::make_shared<SMS_STS>();
        m_wrist_driver->begin(baud, wrist_port.c_str());

        m_head_driver = std::make_shared<SMS_STS>();
        m_head_driver->begin(baud, head_port.c_str());

	sleep(1);
	setZero();
    }

private:
    rclcpp::Subscription<motor_interface::msg::VelAndPose>::SharedPtr wrist_roll_sub_;
    rclcpp::Subscription<motor_interface::msg::VelAndPose>::SharedPtr wrist_pitch_sub_;
    rclcpp::Subscription<motor_interface::msg::VelAndPose>::SharedPtr wrist_yaw_sub_;
    rclcpp::Subscription<motor_interface::msg::VelAndPose>::SharedPtr gripper_pull_sub_;
    rclcpp::Subscription<motor_interface::msg::VelAndPose>::SharedPtr head_pitch_sub_;
    rclcpp::Subscription<motor_interface::msg::VelAndPose>::SharedPtr head_yaw_sub_;
    rclcpp::Subscription<motor_interface::msg::VelAndPose>::SharedPtr camera_pitch_sub_;
    rclcpp::Subscription<motor_interface::msg::VelAndPose>::SharedPtr camera_yaw_sub_;

    std::map<std::string, uint8_t> motor_ids_;

    std::shared_ptr<SMS_STS> m_wrist_driver;
    std::shared_ptr<SMS_STS> m_head_driver;
    int m_id_wrist_roll = 0;
    int m_id_wrist_pitch = 0;
    int m_id_wrist_yaw = 0;
    int m_id_gripper_pull = 0;
    int m_id_head_pitch = 0;
    int m_id_head_yaw = 0;
    int m_id_camera_pitch = 0;
    int m_id_camera_yaw = 0;

    float m_last_head_pitch       = 0.0;
    float m_last_head_yaw         = 0.0;
    float m_last_wrist_roll       = 0.0;
    float m_last_wrist_pitch      = 0.0;
    float m_last_wrist_yaw        = 0.0;
    float m_last_gripper          = 0.0;
    float m_last_camera_pitch     = 0.0;
    float m_last_camera_yaw       = 0.0;

private:
    void loadMotorIDs()
    {
        try
        {
            YAML::Node config = YAML::LoadFile("config/motor_ids.yaml");
            motor_ids_["wrist_roll"] = config["wrist_roll"].as<uint8_t>();
            motor_ids_["wrist_pitch"] = config["wrist_pitch"].as<uint8_t>();
            motor_ids_["wrist_yaw"] = config["wrist_yaw"].as<uint8_t>();
            motor_ids_["head_pitch"] = config["head_pitch"].as<uint8_t>();
            motor_ids_["head_yaw"] = config["head_yaw"].as<uint8_t>();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load motor IDs: %s", e.what());
        }
    }

    void wristRollCallback(const motor_interface::msg::VelAndPose::SharedPtr msg)
    {
        if (msg->mode == 3)
        {
            int step = 2048 - (m_last_wrist_roll + msg->yaw.data) / ANGLE_RESOLUTION;
            m_wrist_driver->WritePosEx(m_id_wrist_roll, step, 110, 50);
            m_last_wrist_roll = m_last_wrist_roll + msg->yaw.data;
        }
        else
        {
            tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            m.getRPY(roll, pitch, yaw);

            int step = 2048 - yaw * (180 / 3.14) / ANGLE_RESOLUTION;
            m_wrist_driver->WritePosEx(m_id_wrist_roll, step, 110, 50);
        }
    }

    void wristPitchCallback(const motor_interface::msg::VelAndPose::SharedPtr msg)
    {
        if (msg->mode == 3)
        {
            int step = 2048 + (m_last_wrist_pitch + msg->yaw.data) / ANGLE_RESOLUTION;
            m_wrist_driver->WritePosEx(m_id_wrist_pitch, step, 110, 50);
            m_last_wrist_pitch = m_last_wrist_pitch + msg->yaw.data;
        }
        else
        {
            tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            m.getRPY(roll, pitch, yaw);

            int step = 2048 + yaw * (180 / 3.14) / ANGLE_RESOLUTION;
            m_wrist_driver->WritePosEx(m_id_wrist_pitch, step, 110, 50);
        }
    }

    void wristYawCallback(const motor_interface::msg::VelAndPose::SharedPtr msg)
    {
        if (msg->mode == 3)
        {
            int step = 2048 + (m_last_wrist_yaw + msg->yaw.data) / ANGLE_RESOLUTION;
            m_wrist_driver->WritePosEx(m_id_wrist_yaw, step, 110, 50);
            m_last_wrist_yaw = m_last_wrist_yaw + msg->yaw.data;
        }
        else
        {
            tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            m.getRPY(roll, pitch, yaw);

            int step = 2048 + yaw * (180 / 3.14) / ANGLE_RESOLUTION;
            m_wrist_driver->WritePosEx(m_id_wrist_yaw, step, 110, 50);
        }
    }

    void gripperPullCallback(const motor_interface::msg::VelAndPose::SharedPtr msg)
    {
        if (msg->mode == 3)
        {
            int step = 2048 + (m_last_gripper + msg->yaw.data) / ANGLE_RESOLUTION;
            m_wrist_driver->WritePosEx(m_id_gripper_pull, step, 1200, 100);
            m_last_gripper = m_last_gripper + msg->yaw.data;
        }
        else
        {
            tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            m.getRPY(roll, pitch, yaw);

            int step = 2048 + yaw * (180 / 3.14) / ANGLE_RESOLUTION;
            m_wrist_driver->WritePosEx(m_id_gripper_pull, step, 1200, 100);
        }
    }

    void headPitchCallback(const motor_interface::msg::VelAndPose::SharedPtr msg)
    {
        if (msg->mode == 3)
        {
            int step = 2048 + (m_last_head_pitch + msg->yaw.data) / ANGLE_RESOLUTION;
            m_head_driver->WritePosEx(m_id_head_pitch, step, 110, 50);
            m_last_head_pitch = m_last_head_pitch + msg->yaw.data;
            RCLCPP_INFO(this->get_logger(), "head_pitch step: %d", step);
        }
        else
        {
            // m_head_driver->WritePosEx(m_id_head_pitch, 2048, 110, 50);
            // int data = m_head_driver->Ping(m_id_head_pitch);
            tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            m.getRPY(roll, pitch, yaw);

            int step = 2048 + yaw * (180 / 3.14) / ANGLE_RESOLUTION;
            m_head_driver->WritePosEx(m_id_head_pitch, step, 110, 50);
            RCLCPP_INFO(this->get_logger(), "head_pitch step: %d", step);
        }
    }

    void headYawCallback(const motor_interface::msg::VelAndPose::SharedPtr msg)
    {
        if (msg->mode == 3)
        {
            int step = 2048 + (m_last_head_yaw + msg->yaw.data) / ANGLE_RESOLUTION;
            m_head_driver->WritePosEx(m_id_head_yaw, step, 110, 50);
            m_last_head_yaw = m_last_head_yaw + msg->yaw.data;
            RCLCPP_INFO(this->get_logger(), "head_yaw step: %d", step);
        }
        else
        {
            // m_head_driver->WritePosEx(m_id_head_yaw, 1000, 110, 50);
            // int data = m_head_driver->Ping(m_id_head_yaw);
        // RCLCPP_INFO(this->get_logger(), "data: %d", data);
            tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            m.getRPY(roll, pitch, yaw);

            int step = 2048 + yaw * (180 / 3.14) / ANGLE_RESOLUTION;
            m_head_driver->WritePosEx(m_id_head_yaw, step, 110, 50);
            RCLCPP_INFO(this->get_logger(), "head_yaw step: %d", step);
        }
    }

    void cameraPitchCallback(const motor_interface::msg::VelAndPose::SharedPtr msg)
    {
        if (msg->mode == 3)
        {
            int step = 2048 + (m_last_camera_pitch + msg->yaw.data) / ANGLE_RESOLUTION;
            m_head_driver->WritePosEx(m_id_camera_pitch, step, 110, 50);
            m_last_camera_pitch = m_last_camera_pitch + msg->yaw.data;
        }
        else
        {
            tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            m.getRPY(roll, pitch, yaw);

            int step = 2048 + yaw * (180 / 3.14) / ANGLE_RESOLUTION;
            m_head_driver->WritePosEx(m_id_camera_pitch, step, 110, 50);
        }
    }

    void cameraYawCallback(const motor_interface::msg::VelAndPose::SharedPtr msg)
    {
        if (msg->mode == 3)
        {
            int step = 2048 + (m_last_camera_yaw + msg->yaw.data) / ANGLE_RESOLUTION;
            m_head_driver->WritePosEx(m_id_camera_yaw, step, 110, 50);
            m_last_camera_yaw = m_last_camera_yaw + msg->yaw.data;
        }
        else
        {
            tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            m.getRPY(roll, pitch, yaw);

            int step = 2048 + yaw * (180 / 3.14) / ANGLE_RESOLUTION;
            m_head_driver->WritePosEx(m_id_camera_yaw, step, 110, 50);
        }
    }

    void sendCommand(const std::string &motor_name, double position, double speed, double acceleration)
    {
        auto it = motor_ids_.find(motor_name);
        if (it != motor_ids_.end())
        {
            uint8_t id = it->second;
            int16_t pos = static_cast<int16_t>(position);
            uint16_t spd = static_cast<uint16_t>(speed);         // 从消息中接收的速度值
            uint8_t acc = static_cast<uint8_t>(acceleration);    // 从消息中接收的加速度值
            
            std::vector<uint8_t> ids = {id};
            std::vector<int16_t> positions = {pos};
            std::vector<uint16_t> speeds = {spd};
            std::vector<uint8_t> accs = {acc};

            // servo_driver_->WritePosEx(ids.data(), positions.data(), speeds.data(), accs.data());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Motor ID not found for %s", motor_name.c_str());
        }
    }

    void setZero()
    {
        m_wrist_driver->WritePosEx(m_id_wrist_roll, 2048, 110, 50);
        m_wrist_driver->WritePosEx(m_id_wrist_pitch, 2048, 110, 50);
        m_wrist_driver->WritePosEx(m_id_wrist_yaw, 2048, 110, 50);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControllerNode>());
    rclcpp::shutdown();
    return 0;
}
