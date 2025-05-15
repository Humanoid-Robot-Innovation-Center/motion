#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <modbus/modbus.h>
#include <string>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>

// 速度/位置模式
enum class MotorMode {
    NONE,
    SPEED_MODE,
    POSITION_MODE
};


class MotorController {
public:
    MotorController(const std::string& device, int baud, int data_bit, int stop_bit, int address);
    ~MotorController();

    // 
    void setSpeed(int l_rpm, int r_rpm);

    void setPosition(double l_r, double r_r);


public:
    // 写单个寄存器
    bool writeSingleRegister(uint16_t address, uint16_t value);

    // 写多个寄存器
    bool writeMultipleRegisters(uint16_t start_address, const std::vector<uint16_t>& values);

    // 读单个寄存器
    bool readRegister(uint16_t address, uint16_t& value);

    // 读多个寄存器
    bool readMultipleRegisters(uint16_t start_address, int count, std::vector<uint16_t>& values);

    // 设置为jog速度模式
    bool setJogMode();

    // 设置为位置模式
    bool setPositionMode();

private:
    // 设备
    std::unique_ptr<modbus_t, decltype(&modbus_free)> m_device;
    
    // 端口号
    std::string m_port = "/ttyUSB0";

    // 波特率
    int m_baud = 115200;

    // 校验方式, 默认无校验
    char m_parity = 'N';
    
    // 数据位, 默认8位
    int m_data_bit = 8;

    // 停止位, 默认1位
    int m_stop_bit = 1;

private:
    // 日志模块
    rclcpp::Logger logger_;

    // 状态
    MotorMode m_mode = MotorMode::NONE;

    bool m_have_write_rpm = false;
};

#endif // MOTOR_CONTROLLER_HPP
