#include "rclcpp/rclcpp.hpp"
#include "ethercat_controller/msg/position_command.hpp"
#include "ethercat_controller/msg/speed_command.hpp"
#include "ethercat_controller/msg/torque_command.hpp"

#include <vector>
#include <algorithm>
#include <chrono>
#include <mutex>

// 使用 std 命名空间简化代码
using namespace std::chrono_literals;

// 假设 EtherCAT_Msg 和相关函数已定义
struct EtherCAT_Msg {
    // 定义您的消息结构
};

// 假设以下函数已实现
int EtherCAT_Init(const char* ifname);
void EtherCAT_Shutdown();
void set_motor_position(EtherCAT_Msg* msg, uint16_t motor_id, int passage, int param1, int param2, int param3, int param4);
void set_motor_speed(EtherCAT_Msg* msg, uint16_t motor_id, int passage, float speed, uint16_t current, int param);
void set_motor_torque(EtherCAT_Msg* msg, uint16_t motor_id, int passage, int16_t torque);
int ec_send_processdata();
int ec_receive_processdata(int timeout);

struct MotorMapping {
    uint16_t motor_id;
    int slave_index;
    int passage;
};

// 全局映射表
std::vector<MotorMapping> motor_mappings = {
    {14, 3, 1},
    {18, 4, 2},
    {3, 5, 1}
    // 添加更多映射关系
};

class EtherCATController : public rclcpp::Node
{
public:
    EtherCATController() : Node("ethercat_controller")
    {
        // 初始化 EtherCAT
        if (EtherCAT_Init("enp3s0") != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize EtherCAT.");
            rclcpp::shutdown();
            return;
        }

        // 初始化 Tx_Message 和 isConfig
        Tx_Message.resize(ec_slavecount, EtherCAT_Msg());
        isConfig.resize(ec_slavecount, false);

        // 订阅 ROS 2 消息
        position_subscription_ = this->create_subscription<ethercat_controller::msg::PositionCommand>(
            "position_command", 10,
            std::bind(&EtherCATController::position_command_callback, this, std::placeholders::_1));

        speed_subscription_ = this->create_subscription<ethercat_controller::msg::SpeedCommand>(
            "speed_command", 10,
            std::bind(&EtherCATController::speed_command_callback, this, std::placeholders::_1));

        torque_subscription_ = this->create_subscription<ethercat_controller::msg::TorqueCommand>(
            "torque_command", 10,
            std::bind(&EtherCATController::torque_command_callback, this, std::placeholders::_1));

        // 定时器：定期发送和接收过程数据
        timer_ = this->create_wall_timer(
            10ms, std::bind(&EtherCATController::send_receive_process_data, this));
    }

    ~EtherCATController()
    {
        EtherCAT_Shutdown();
    }

private:
    // 位置命令回调
    void position_command_callback(const ethercat_controller::msg::PositionCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Setting motor %d position to %f", msg->motor_id, msg->position);
        EtherCAT_Command_Set(msg->motor_id, msg->position, msg->speed, msg->current);
    }

    // 速度命令回调
    void speed_command_callback(const ethercat_controller::msg::SpeedCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Setting motor %d speed to %f", msg->motor_id, msg->speed);
        EtherCAT_Command_Set_Speed(msg->motor_id, msg->speed, msg->current);
    }

    // 力矩命令回调
    void torque_command_callback(const ethercat_controller::msg::TorqueCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Setting motor %d torque to %d", msg->motor_id, msg->torque);
        EtherCAT_Command_Set_Torque(msg->motor_id, msg->torque);
    }

    // 发送和接收过程数据
    void send_receive_process_data()
    {
        std::lock_guard<std::mutex> lock(tx_mutex);
        ec_send_processdata();
        int wkc = ec_receive_processdata(EC_TIMEOUTRET);
        if (wkc < expectedWKC) {
            RCLCPP_ERROR(this->get_logger(), "EtherCAT process data error: Bad WKC (%d)", wkc);
        }
    }

    // 设置位置命令
    void EtherCAT_Command_Set(uint16_t motor_id, float position, uint16_t speed, uint16_t current)
    {
        std::lock_guard<std::mutex> lock(tx_mutex);
        auto it = std::find_if(motor_mappings.begin(), motor_mappings.end(),
                               [motor_id](const MotorMapping& mapping) { return mapping.motor_id == motor_id; });

        if (it == motor_mappings.end()) {
            RCLCPP_ERROR(this->get_logger(), "Motor ID %d not found in mapping!", motor_id);
            return;
        }

        int slave = it->slave_index;
        int passage = it->passage;

        if (slave >= ec_slavecount) {
            RCLCPP_ERROR(this->get_logger(), "Slave index %d out of range!", slave);
            return;
        }

        // 设置电机位置命令
        set_motor_position(&Tx_Message[slave], it->motor_id, passage, 0, 200, 50, 2);

        // 更新从站输出
        EtherCAT_Msg *slave_dest = (EtherCAT_Msg *)(ec_slave[slave + 1].outputs);
        if (slave_dest) {
            *slave_dest = Tx_Message[slave];
            isConfig[slave] = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get outputs for slave %d", slave + 1);
        }
    }

    // 设置速度命令
    void EtherCAT_Command_Set_Speed(uint16_t motor_id, float speed, uint16_t current)
    {
        std::lock_guard<std::mutex> lock(tx_mutex);
        auto it = std::find_if(motor_mappings.begin(), motor_mappings.end(),
                               [motor_id](const MotorMapping& mapping) { return mapping.motor_id == motor_id; });

        if (it == motor_mappings.end()) {
            RCLCPP_ERROR(this->get_logger(), "Motor ID %d not found in mapping!", motor_id);
            return;
        }

        int slave = it->slave_index;
        int passage = it->passage;

        if (slave >= ec_slavecount) {
            RCLCPP_ERROR(this->get_logger(), "Slave index %d out of range!", slave);
            return;
        }

        // 设置电机速度命令
        set_motor_speed(&Tx_Message[slave], it->motor_id, passage, speed, current, 2);

        // 更新从站输出
        EtherCAT_Msg *slave_dest = (EtherCAT_Msg *)(ec_slave[slave + 1].outputs);
        if (slave_dest) {
            *slave_dest = Tx_Message[slave];
            isConfig[slave] = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get outputs for slave %d", slave + 1);
        }
    }

    // 设置力矩命令
    void EtherCAT_Command_Set_Torque(uint16_t motor_id, int16_t torque)
    {
        std::lock_guard<std::mutex> lock(tx_mutex);
        auto it = std::find_if(motor_mappings.begin(), motor_mappings.end(),
                               [motor_id](const MotorMapping& mapping) { return mapping.motor_id == motor_id; });

        if (it == motor_mappings.end()) {
            RCLCPP_ERROR(this->get_logger(), "Motor ID %d not found in mapping!", motor_id);
            return;
        }

        int slave = it->slave_index;
        int passage = it->passage;

        if (slave >= ec_slavecount) {
            RCLCPP_ERROR(this->get_logger(), "Slave index %d out of range!", slave);
            return;
        }

        // 设置电机力矩命令
        set_motor_torque(&Tx_Message[slave], it->motor_id, passage, torque);

        // 更新从站输出
        EtherCAT_Msg *slave_dest = (EtherCAT_Msg *)(ec_slave[slave + 1].outputs);
        if (slave_dest) {
            *slave_dest = Tx_Message[slave];
            isConfig[slave] = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get outputs for slave %d", slave + 1);
        }
    }

    // 订阅者
    rclcpp::Subscription<ethercat_controller::msg::PositionCommand>::SharedPtr position_subscription_;
    rclcpp::Subscription<ethercat_controller::msg::SpeedCommand>::SharedPtr speed_subscription_;
    rclcpp::Subscription<ethercat_controller::msg::TorqueCommand>::SharedPtr torque_subscription_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // EtherCAT 相关
    std::vector<EtherCAT_Msg> Tx_Message;
    std::vector<bool> isConfig;

    // 互斥锁
    std::mutex tx_mutex;

    // 预期的 WKC（工作计数），根据您的配置进行调整
    static constexpr int expectedWKC = 4; // 示例值
};

// 主函数
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EtherCATController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
