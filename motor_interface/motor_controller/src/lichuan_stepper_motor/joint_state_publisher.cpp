#include "JointStatePublisher.hpp"

JointStatePublisher::JointStatePublisher() 
    : Node("joint_state_publisher") {
    // 创建发布者
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // 创建定时器，每0.1秒发布一次
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { publish_joint_state(); }
    );

    // 初始化关节名称和位置
    joint_state_.name = {"joint_1", "joint_2", "joint_3"}; // 根据实际情况修改
    joint_state_.position.resize(joint_state_.name.size(), 0.0);
}

void JointStatePublisher::publish_joint_state() {
    // 这里应该有读取电机反馈的逻辑
    for (size_t i = 0; i < joint_state_.position.size(); ++i) {
        joint_state_.position[i] = get_motor_feedback(i); // 替换为实际电机反馈函数
    }

    // 发布关节状态
    joint_state_.header.stamp = this->now();
    joint_state_publisher_->publish(joint_state_);
    RCLCPP_INFO(this->get_logger(), "Published joint states: [%f, %f, %f]", 
                joint_state_.position[0], joint_state_.position[1], joint_state_.position[2]);
}

double JointStatePublisher::get_motor_feedback(size_t joint_index) {
    // 示例：返回模拟的电机反馈，替换为实际获取电机数据的代码
    return joint_index * 0.1; // 例如，返回0.0, 0.1, 0.2等
}
