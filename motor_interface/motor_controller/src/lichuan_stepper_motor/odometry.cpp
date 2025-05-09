#include <memory>
#include <thread>
#include <chrono>
#include <iostream>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ThreadedNode : public rclcpp::Node
{
public:
    ThreadedNode()
        : Node("threaded_node"), shared_data_("Initial data")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10,
            std::bind(&ThreadedNode::listener_callback, this, std::placeholders::_1));

        // 启动发布线程
        publish_thread_ = std::thread(&ThreadedNode::publish_messages, this);
    }

    ~ThreadedNode()
    {
        running_ = false;  // 停止线程
        if (publish_thread_.joinable()) {
            publish_thread_.join();  // 等待线程结束
        }
    }

private:
    void publish_messages()
    {
        while (rclcpp::ok() && running_) {
            std::unique_lock<std::mutex> lock(mutex_);  // 上锁
            auto msg = std_msgs::msg::String();
            msg.data = shared_data_;  // 使用共享变量
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published: '%s'", msg.data.c_str());
            std::this_thread::sleep_for(std::chrono::seconds(1));  // 发布频率
            // 锁会在这里自动解锁
        }
    }

    void listener_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        std::unique_lock<std::mutex> lock(mutex_);  // 上锁
        shared_data_ = msg->data;  // 更新共享变量
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
        // 锁会在这里自动解锁
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::thread publish_thread_;
    bool running_ = true;  // 控制线程运行的标志
    std::string shared_data_;  // 共享变量
    mutable std::mutex mutex_;  // 互斥量
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThreadedNode>());
    rclcpp::shutdown();
    return 0;
}