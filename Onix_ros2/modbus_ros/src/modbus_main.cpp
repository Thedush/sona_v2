#include "modbus_ros/modbus_ros.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Modbus>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

