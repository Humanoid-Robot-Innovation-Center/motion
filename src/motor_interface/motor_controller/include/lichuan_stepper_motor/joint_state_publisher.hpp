#ifndef JOINT_STATE_PUBLISHER_HPP
#define JOINT_STATE_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStatePublisher : public rclcpp::Node {
public:
    JointStatePublisher();

private:
    void publish_joint_state();
    double get_motor_feedback(size_t joint_index);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_;
};

#endif // JOINT_STATE_PUBLISHER_HPP
