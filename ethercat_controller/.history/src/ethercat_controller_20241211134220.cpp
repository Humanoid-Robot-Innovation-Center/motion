#include <vector>
#include <mutex>
#include <algorithm>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

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
    : Node("ethercat_controller")
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
        command_subscription_ = this->create_subscription<your_custom_msg_package::msg::ControlCommand>(
            "control_command", 10, std::bind(&EtherCATController::control_command_callback, this, std::placeholders::_1));

        // 创建定时器，以模拟原有的主循环
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&EtherCATController::timer_callback, this));
    }

private:
    void control_command_callback(const your_custom_msg_package::msg::ControlCommand::SharedPtr msg)
    {
        // 解析命令并调用EtherCAT命令设置函数
        process_command(msg);
        ec_send_processdata();
    }

    void process_command(const your_custom_msg_package::msg::ControlCommand::SharedPtr msg)
    {
        // 根据收到的命令和全局映射表设置EtherCAT消息
        for (const auto& mapping : motor_mappings) {
            if (msg->motor_id == mapping.motor_id) {
                if (msg->control_mode == 0) { // 位置控制
                    set_motor_position(&Tx_Message[mapping.slave_index], 
                                       mapping.passage, 
                                       msg->motor_id, 
                                       msg->position, 
                                       200, // 示例速度
                                       msg->current, 
                                       2);   // 示例ack_status
                } 
                else if (msg->control_mode == 1) { // 速度控制
                    set_motor_speed(&Tx_Message[mapping.slave_index], 
                                    mapping.passage, 
                                    msg->motor_id, 
                                    msg->speed, 
                                    msg->current, 
                                    2);   // 示例ack_status
                } 
                else if (msg->control_mode == 2) { // 力矩控制
                    set_motor_cur_tor(&Tx_Message[mapping.slave_index], 
                                      mapping.passage, 
                                      msg->motor_id, 
                                      msg->torque, 
                                      0, // 示例ctrl_status
                                      1); // 示例ack_status
                }
             
                // 将消息写回EtherCAT
       // 更新从站输出
        isConfig[slave] = true;
        EtherCAT_Msg *slave_dest = (EtherCAT_Msg *)(ec_slave[slave + 1].outputs);
        if (slave_dest) {
            *slave_dest = Tx_Message[slave];

        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get outputs for slave %d", slave + 1);
        }
    }
            }
        }
    }


