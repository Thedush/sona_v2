#include "modbus_driver/rfid_driver.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RfidModbusDriver>());
    rclcpp::shutdown();
    return 0;
}