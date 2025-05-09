#ifndef JOINT_STATE_PUBLISHER_HPP
#define JOINT_STATE_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <motor_interface/srv/get_joint_state.hpp>
#include <string>
#include "lichuan_stepper_motor/motor_controller.hpp"
#include "motor_interface/msg/vel_and_pose.hpp"
#include "feetech_servo_motor/SMS_STS.h"

struct JointState
{
    std::string name = "";     // 关节名称
    double position  = 0.0;    // 位置
    double velocity  = 0.0;    // 速度
    double effort    = 0.0;    // 力矩
};


class JointStatePublisher : public rclcpp::Node
{
public:
    JointStatePublisher();

private:
    void handle_get_joint_state(const std::shared_ptr<motor_interface::srv::GetJointState::Request> request,
        std::shared_ptr<motor_interface::srv::GetJointState::Response> response);

    double get_motor_feedback(size_t joint_index);

    void headPitchCallback(JointState& state);

    void headYawCallback(JointState& state);

    void wristRollCallback(JointState& state);

    void wristPitchCallback(JointState& state);

    void wristYawCallback(JointState& state);

    void gripperPullCallback(JointState& state);

    void liftCallback(JointState& state);
    
    void armCallback(JointState& state);


private:
    rclcpp::Service<motor_interface::srv::GetJointState>::SharedPtr joint_state_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    

    std::shared_ptr<MotorController> m_base_controller;               // 底盘驱动轮控制器
    std::shared_ptr<MotorController> m_lift_controller;               // 升降控制器
    std::shared_ptr<MotorController> m_arm_controller;                // 伸缩控制器

    double m_base_transmission_ratio = 1.0;     // 底盘传动比
    double m_lift_transmission_ratio = 1.0;     // 升降传动比
    double m_arm_transmission_ratio = 1.0;      // 伸缩传动比

    std::shared_ptr<SMS_STS> m_wrist_driver;
    std::shared_ptr<SMS_STS> m_head_driver;

    int m_id_wrist_roll = 0;
    int m_id_wrist_pitch = 0;
    int m_id_wrist_yaw = 0;
    int m_id_gripper_pull = 0;
    int m_id_head_pitch = 0;
    int m_id_head_yaw = 0;
    int m_id_head_camera_yaw   = 0;
    int m_id_head_camera_pitch = 0;
};

#endif // JOINT_STATE_PUBLISHER_HPP
