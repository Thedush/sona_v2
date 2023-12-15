#ifndef MODBUS_DRIVER_H
#define MODBUS_DRIVER_H

#include "modbus_driver/modbus.h"
#include "modbus_driver/modbus-rtu.h"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "onix_msgs/msg/drive.hpp"

class RfidModbusDriver : public rclcpp::Node
{
public:
    RfidModbusDriver();
    ~RfidModbusDriver();
    bool sensorstatus;
    uint16_t stop_robot_reg ;

private:
    void delay(int milliseconds);
    void publish();
    void drive_callback();
    // void stop_robot_callback(const std_msgs::msg::Bool::SharedPtr msg);
    // void battery_callback(const std_msgs::msg::Float32MultiArray::SharedPtr battery_msg);
    // int counter;
    // int battery_percentage,charge_status;

    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr SonarFrontpublisher;
    // rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr SonarRearpublisher;
    // rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr MagneticTapePublisher;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr RfidPublisher;
    // rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr PLCPublisher;
    // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr BatterySub;
    // rclcpp::Subscription<onix_msgs::msg::Drive>::SharedPtr driver_sub;
    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_robot;
    modbus_t *ctx ;
};

#endif // MODBUS_DRIVER_H

