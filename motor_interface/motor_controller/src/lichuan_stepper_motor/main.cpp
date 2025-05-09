#include <rclcpp/rclcpp.hpp>
#include "lichuan_stepper_motor/driver_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriverNode>());
    rclcpp::shutdown();
    return 0;
}