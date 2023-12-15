#ifndef MODBUS_DRIVER_H
#define MODBUS_DRIVER_H

#include "modbus_driver/modbus.h"
#include "modbus_driver/modbus-rtu.h"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "onix_msgs/msg/drive.hpp"
class ModbusDriver : public rclcpp::Node
{
public:
    ModbusDriver();
    ~ModbusDriver();
    bool sensorstatus;
    uint16_t stop_robot_reg ;

private:
    void delay(int milliseconds);
    void publish();
    void drive_callback(const onix_msgs::msg::Drive::SharedPtr drive_msg);
    void stop_robot_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void battery_callback(const std_msgs::msg::Float32MultiArray::SharedPtr battery_msg);
    double accelerateDecelerate(double previous_value, double target_value,double acceleration, double deceleration);
    int counter;
    int battery_percentage,charge_status;
    double acceleration = 0.2;  // Adjust this value as desired
    double deceleration = 0.2;  // Adjust this value as desired
    double previous_left_wheel = 0.0;
    double previous_right_wheel = 0.0;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr SonarFrontpublisher;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr SonarRearpublisher;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr MagneticTapePublisher;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr RfidPublisher;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr PLCPublisher;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr BatterySub;
    rclcpp::Subscription<onix_msgs::msg::Drive>::SharedPtr driver_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_robot;
    modbus_t *ctx ;
};

#endif // MODBUS_DRIVER_H

