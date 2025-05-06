#include <vector>
#include <mutex>
#include <algorithm>
#include <cstdint>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include "Console.hpp"
#include "command.h"
#include "time.h"
extern "C" {
#include "ethercat.h"
}

#include "emotor_interfaces/msg/ControlCommand.hpp> // 替换为你的消息包和消息类型

struct MotorMapping {
    uint16_t motor_id;
    int slave_index; // EtherCAT slave index
    int passage;     // EtherCAT passage
};

// 全局映射表
std::vector<MotorMapping> motor_mappings = {
    {14, 3, 1}, // Motor ID 14 -> Slave 3, Passage 1
    {18, 4, 2}, // Motor ID 18 -> Slave 4, Passage 2
    {3, 5, 1}   // Motor ID 3 -> Slave 5, Passage 1
};

// 互斥锁保护映射表的访问
std::mutex mapping_mutex;

class EtherCATController : public rclcpp::Node
{
public:
    EtherCATController() 
    : Node("ethercat_controller"), last_command_(std::nullopt)
    {
        // 初始化 EtherCAT
        if (EtherCAT_Init("enp3s0") != 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "未找到从站, 程序退出！");
            rclcpp::shutdown();
            return;
        }
        else
            RCLCPP_INFO(this->get_logger(), "从站数量： %d", ec_slavecount);
            
        // 创建订阅器
        command_subscription_ = this->create_subscription<emotor_interfaces::msg::ControlCommand>(
            "control_command", 10, std::bind(&EtherCATController::control_command_callback, this, std::placeholders::_1));

        // 创建定时器，以模拟原有的主循环
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&EtherCATController::timer_callback, this));
    }

private:
    std::optional<emotor_interfaces::msg::ControlCommand::SharedPtr> last_command_;

    void control_command_callback(const emotor_interfaces::msg::ControlCommand::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mapping_mutex);
        last_command_ = msg; // 更新最后一次接收到的命令
    }

    void timer_callback()
    {
        if (last_command_) {
            process_command(*last_command_); // 使用最后一次的命令进行处理
            ec_send_processdata();
        }
    }

    void process_command(const emotor_interfaces::msg::ControlCommand::SharedPtr msg)
    {
        for (const auto& mapping : motor_mappings) {
            if (msg->motor_id == mapping.motor_id) {
                // 根据控制模式设置相应的命令
                switch (msg->control_mode) {
                    case 0: // 位置控制
                        set_motor_position(&Tx_Message[mapping.slave_index], 
                                           mapping.passage, 
                                           msg->motor_id, 
                                           msg->position, 
                                           200, // 示例速度
                                           msg->current, 
                                           2);   // 示例ack_status
                        break;
                    case 1: // 速度控制
                        set_motor_speed(&Tx_Message[mapping.slave_index], 
                                        mapping.passage, 
                                        msg->motor_id, 
                                        msg->speed, 
                                        msg->current, 
                                        2);   // 示例ack_status
                        break;
                    case 2: // 力矩控制
                        set_motor_cur_tor(&Tx_Message[mapping.slave_index], 
                                          mapping.passage, 
                                          msg->motor_id, 
                                          msg->torque, 
                                          0, // 示例ctrl_status
                                          1); // 示例ack_status
                        break;
                }

                // 更新从站输出
                EtherCAT_Msg *slave_dest = (EtherCAT_Msg *)(ec_slave[mapping.slave_index + 1].outputs);
                if (slave_dest) {
                    *slave_dest = Tx_Message[mapping.slave_index];
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get outputs for slave %d", mapping.slave_index + 1);
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EtherCATController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
