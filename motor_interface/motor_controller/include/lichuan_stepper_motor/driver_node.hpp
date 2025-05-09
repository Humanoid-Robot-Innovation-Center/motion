#ifndef DRIVER_NODE_HPP
#define DRIVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lichuan_stepper_motor/motor_controller.hpp"
#include "motor_interface/msg/vel_and_pose.hpp"
#include <thread>

class DriverNode : public rclcpp::Node
{
public:
    DriverNode();

    virtual ~DriverNode();

    double getWheelBase()
    {
        std::lock_guard<std::mutex> lock(m_wheel_base_mutex);
        return m_wheel_base;
    }

    double getWheelRadius()
    {
        std::lock_guard<std::mutex> lock(m_wheel_radius_mutex);
        return m_wheel_radius;
    }

    double getBaseReductionRatio()
    {
        std::lock_guard<std::mutex> lock(m_base_reduction_ratio_mutex);
        return m_base_reduction_ratio;
    }

    double getLiftReductionRatio()
    {
        std::lock_guard<std::mutex> lock(m_lift_reduction_ratio_mutex);
        return m_lift_reduction_ratio;
    }

    double getArmReductionRatio()
    {
        std::lock_guard<std::mutex> lock(m_arm_reduction_ratio_mutex);
        return m_arm_reduction_ratio;
    }

private:
    /* topic: */
    rclcpp::Subscription<motor_interface::msg::VelAndPose>::SharedPtr m_base_vel_pose_sub;      // 底盘驱动-订阅速度和位姿
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_base_vel_sub;                  // 底盘驱动-订阅速度
    rclcpp::Subscription<motor_interface::msg::VelAndPose>::SharedPtr m_lift_sub;               // 升降-订阅位姿和速度
    rclcpp::Subscription<motor_interface::msg::VelAndPose>::SharedPtr m_arm_sub;                // 伸缩-订阅位姿和速度
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    /* 控制器: */
    std::shared_ptr<MotorController> m_base_controller;               // 底盘驱动轮控制器
    std::shared_ptr<MotorController> m_lift_controller;               // 手臂升降控制器
    std::shared_ptr<MotorController> m_arm_controller;                // 手臂伸缩控制器

    /* 参数: */
    std::string m_device_port     = "";        // 设备端口
    int m_baud                    = 0;         // 波特率
    int m_data_bit                = 0;         // 数据位
    int m_stop_bit                = 0;         // 停止位
    int m_base_slave_id           = 0;         // 底盘站号
    int m_lift_slave_id           = 0;         // 升降站号
    int m_arm_slave_id            = 0;         // 伸缩站号
    double m_base_reduction_ratio = 1.0;       // 底盘传动比
    double m_lift_reduction_ratio = 1.0;       // 升降传动比
    double m_arm_reduction_ratio  = 1.0;       // 伸缩传动比
    double m_wheel_base           = 0.0;       // 底盘轮间距
    double m_wheel_radius         = 0.0;       // 底盘轮半径

    /* 线程: */
    std::thread m_odom_thread; // 处理里程计的线程
    mutable std::mutex m_controller_mutex; // 互斥量
    std::mutex m_wheel_base_mutex;
    std::mutex m_wheel_radius_mutex;
    std::mutex m_base_reduction_ratio_mutex;
    std::mutex m_lift_reduction_ratio_mutex;
    std::mutex m_arm_reduction_ratio_mutex;

    rclcpp::Clock m_clock;
    rclcpp::Time m_last_time = this->get_clock()->now();
    double m_current_x = 0.0;
    double m_current_y = 0.0;
    double m_current_theta = 0.0;

    double m_last_arm_pose = 0.0;
    bool m_is_arm_pose_increase = false;
    double m_delta_arm_pose = 0.0;

    double m_last_pose_x = 0.0;
    double m_last_pose_y = 0.0;
    double m_last_x = 0.0;
    double m_last_y = 0.0;
    double m_last_z = 0.0;
    double m_last_w = 1.0;

    double m_last_vel = 0.0;

    bool m_haveSetVelMode = false;

    int m_lift_times = 0.0;
private:
    void handBaseVelAndPose(const motor_interface::msg::VelAndPose::SharedPtr msg);

    void handBaseVel(const geometry_msgs::msg::Twist::SharedPtr msg);

    void handLift(const motor_interface::msg::VelAndPose::SharedPtr msg);

    void handArm(const motor_interface::msg::VelAndPose::SharedPtr msg);

    void publishOdom();

    void setZero();

    void test()
    {
        int a = 0;
    }
};

#endif // DRIVER_NODE_HPP
