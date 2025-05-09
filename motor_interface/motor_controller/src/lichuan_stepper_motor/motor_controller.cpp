#include "lichuan_stepper_motor/motor_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <vector>

/*
* 底盘左轮步进电机-ID:21 设备地址:1
* 底盘右轮步进电机-ID:21 设备地址:2
* 身子升降步进电机-ID:22 设备地址:1
* 手臂伸缩步进电机-ID:22 设备地址:2
*/

#define AXIS1_DI_FUNC_ADDRESS 0x0022                // 轴1 DI1功能选择的地址(PA_029)
#define AXIS2_DI_FUNC_ADDRESS 0x0122                // 轴2 DI1功能选择的地址(PA_068)
#define AXIS1_DI_LOGIC_ADDRESS 0x001F               // 轴1 DI1输入逻辑的地址(PA_026)
#define AXIS2_DI_LOGIC_ADDRESS 0x011F               // 轴2 DI1输入逻辑的地址(PA_065)
#define DI_POSE_MODE 0x20                           // DI功能位置模式(0x20 - 32d)
#define DI_SPEED_MODE 0x23                          // DI功能速度模式(0x23 - 35d)
#define DI_LOGIC_0 0x00                             // DI逻辑状态0
#define DI_LOGIC_1 0x01                             // DI逻辑状态1

#define AXIS1_JOG_SPEED_ADDRESS 2009                // 轴1 JOG速度的地址(PA_108)
#define AXIS2_JOG_SPEED_ADDRESS 2409                // 轴1 JOG速度的地址(PA_193)


#define AXIS1_CONTROL_ADDRESS 0x2002                // 轴1 运行控制字的地址(PA_101)
#define AXIS2_CONTROL_ADDRESS 0x2402                // 轴2 运行控制字的地址(PA_186)
#define CONTROL_START 0x10                          // 控制字-启动
#define CONTROL_STOP 0x00                           // 控制字-停止
#define AXIS1_POSE_MODE_SPEED_ADDRESS 0x2203        // 轴1 位置模式下的速度
#define AXIS2_POSE_MODE_SPEED_ADDRESS 0x2603        // 轴2 位置模式下的速度

int32_t calculateComplement(int32_t num)
{
    return ~num + 1;
}

// 构造函数
MotorController::MotorController(const std::string& device, int baud, int data_bit, int stop_bit, int address)
    : m_port(device), m_baud(baud), m_data_bit(data_bit), m_stop_bit(stop_bit), logger_(rclcpp::get_logger("motor_controller")),
      m_device(nullptr, &modbus_free)  // 使用自定义删除器
{
    // 初始化
    m_device.reset(modbus_new_rtu(m_port.c_str(), m_baud, m_parity, m_data_bit, m_stop_bit));
    if (!m_device)
    {
        RCLCPP_ERROR(logger_, "Unable to create Modbus context");
    }

    // 设置从站地址
    if (modbus_set_slave(m_device.get(), address) == -1)
    {
        RCLCPP_ERROR(logger_, "address: %d; Failed to set slave address: %s", address, modbus_strerror(errno));
    }

    // 连接到从站设备
    if (modbus_connect(m_device.get()) == -1)
    {
        RCLCPP_ERROR(logger_, "Connection failed: %s; device: %s", modbus_strerror(errno), device.c_str());
    }
}

// 析构函数
MotorController::~MotorController()
{
    modbus_close(m_device.get());
    modbus_free(m_device.get());
}


// 设置jog速度模式
bool MotorController::setJogMode()
{
    // 参考: DI口功能配置说明、DI功能命令表、参数一览表
    // 轴1和轴2的功能位、输入极性逻辑
    if (writeSingleRegister(AXIS1_DI_FUNC_ADDRESS, DI_SPEED_MODE) && writeSingleRegister(AXIS1_DI_LOGIC_ADDRESS, DI_LOGIC_1) &&
        writeSingleRegister(AXIS2_DI_FUNC_ADDRESS, DI_SPEED_MODE) && writeSingleRegister(AXIS2_DI_LOGIC_ADDRESS, DI_LOGIC_1))
    {
        RCLCPP_INFO(logger_, "setJogMode successful");
        m_mode = MotorMode::SPEED_MODE;
        return true;
    }
    else
    {
        RCLCPP_ERROR(logger_, "setJogMode failed");
        return false;
    }
}

// 设置位置模式
bool MotorController::setPositionMode()
{
    // 参考: DI口功能配置说明、DI功能命令表、参数一览表
    // 轴1和轴2的功能位、输入极性逻辑
    if (writeSingleRegister(AXIS1_DI_FUNC_ADDRESS, DI_POSE_MODE) && writeSingleRegister(AXIS1_DI_LOGIC_ADDRESS, DI_LOGIC_1) &&
        writeSingleRegister(AXIS2_DI_FUNC_ADDRESS, DI_POSE_MODE) && writeSingleRegister(AXIS2_DI_LOGIC_ADDRESS, DI_LOGIC_1))
    {
        RCLCPP_INFO(logger_, "setPoseMode successful");
        m_mode = MotorMode::POSITION_MODE;
        return true;
    }
    else
    {
        RCLCPP_ERROR(logger_, "setPoseMode failed");
        return false;
    }
}

// 设置速度
void MotorController::setSpeed(int l_rpm, int r_rpm)
{
    // 设置速度
    writeSingleRegister(0x2009, std::abs(l_rpm));
    writeSingleRegister(0x2409, std::abs(r_rpm));

    // 启动
    writeSingleRegister(0x2002, l_rpm > 0 ? 0x58 : 0x38);
    writeSingleRegister(0x2402, r_rpm > 0 ? 0x38 : 0x58);
}

// 设置位置
// revolutions转数
void MotorController::setPosition(double l_r, double r_r)
{
    l_r = l_r * 4000;
    r_r = -r_r * 4000;
    RCLCPP_INFO(logger_, "l_L: %x; l_H: %x; r_L: %x; r_H: %x", calculateComplement(l_r) & 0xFFFF, (calculateComplement(l_r) >> 16) & 0xFFFF,
        calculateComplement(r_r) & 0xFFFF, (calculateComplement(r_r) >> 16) & 0xFFFF);
    writeSingleRegister(0x2002, 0x00);    // 复位
    writeSingleRegister(0x2402, 0x00);    // 复位

    writeSingleRegister(0x2201, (calculateComplement(l_r) & 0xFFFF));
    writeSingleRegister(0x2202, ((calculateComplement(l_r) >> 16) & 0xFFFF));
    writeSingleRegister(0x2200, 0x80);    // 相对位置运行
    writeSingleRegister(0x2203, 0x19);    // 转速100r/min

    writeSingleRegister(0x2601, (calculateComplement(r_r) & 0xFFFF));
    writeSingleRegister(0x2602, ((calculateComplement(r_r) >> 16) & 0xFFFF));
    writeSingleRegister(0x2600, 0x80);    // 相对位置运行
    writeSingleRegister(0x2603, 0x19);    // 转速100r/min

    writeSingleRegister(0x2002, 0x10);    // 启动
    writeSingleRegister(0x2402, 0x10);    // 启动
}

// 写单个寄存器
bool MotorController::writeSingleRegister(uint16_t address, uint16_t value)
{
    if (modbus_write_register(m_device.get(), address, value) == -1)
    {
        RCLCPP_ERROR(logger_, "Failed to write single register: %s", modbus_strerror(errno));
        return false;
    }
    return true;
}

// 写多个寄存器
bool MotorController::writeMultipleRegisters(uint16_t start_address, const std::vector<uint16_t>& values)
{
    if (modbus_write_registers(m_device.get(), start_address, values.size(), values.data()) == -1)
    {
        RCLCPP_ERROR(logger_, "Failed to write multiple registers: %s", modbus_strerror(errno));
        return false;
    }
    return true;
}

// 读单个寄存器
bool MotorController::readRegister(uint16_t address, uint16_t& value)
{
    uint16_t reg;
    if (modbus_read_registers(m_device.get(), address, 1, &reg) == -1)
    {
        RCLCPP_ERROR(logger_, "Failed to read single register: %s", modbus_strerror(errno));
        return false;
    }
    value = reg;
    return true;
}

// 读多个寄存器
bool MotorController::readMultipleRegisters(uint16_t start_address, int count, std::vector<uint16_t>& values)
{
    values.resize(count);
    if (modbus_read_registers(m_device.get(), start_address, count, values.data()) == -1)
    {
        RCLCPP_ERROR(logger_, "Failed to read multiple registers: %s", modbus_strerror(errno));
        return false;
    }
    return true;
}
