#ifndef MODBUS_H_
#define MODBUS_H_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include "agv_srv/srv/modbus_normal.hpp"
#include "modbus_ros/modbus.h"
#include "modbus_ros/modbus-rtu.h"

class Modbus : public rclcpp::Node
{
public:
    Modbus();

    

private:
    modbus_t *ctx ;
    uint16_t read_register[64];
    uint8_t read_coil[64];
    // const uint8_t write_bits;
    void delay(int milliseconds);
    rclcpp::Service<agv_srv::srv::ModbusNormal>::SharedPtr service_;
    bool handle_service(const std::shared_ptr<agv_srv::srv::ModbusNormal::Request> request,
                        std::shared_ptr<agv_srv::srv::ModbusNormal::Response> response);
};

#endif /* MODBUS_H_ */

