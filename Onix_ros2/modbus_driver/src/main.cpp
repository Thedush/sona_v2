#include "modbus_driver/modbus_driver.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModbusDriver>());
    rclcpp::shutdown();
    return 0;
}
