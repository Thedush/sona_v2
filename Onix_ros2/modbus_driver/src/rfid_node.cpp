#include "modbus_driver/rfid_driver.h"
#include <cmath>
using namespace std;

RfidModbusDriver::RfidModbusDriver() : Node("rfid_publisher")
{
    // counter = 0;
    // SonarFrontpublisher = this->create_publisher<std_msgs::msg::Int16MultiArray>("sonar_f_data", 10);
    // SonarRearpublisher = this->create_publisher<std_msgs::msg::Int16MultiArray>("sonar_r_data", 10);
    // MagneticTapePublisher = this->create_publisher<std_msgs::msg::Int16MultiArray>("magnetic_tape_data", 10);
    RfidPublisher = this->create_publisher<std_msgs::msg::Int16MultiArray>("rfid_tape_data", 10);
    // PLCPublisher = this->create_publisher<std_msgs::msg::Int16MultiArray>("plc_data", 10);
    // driver_sub = this->create_subscription<onix_msgs::msg::Drive>("/cmd_drive", rclcpp::SensorDataQoS(),
    //                                                               std::bind(&RfidModbusDriver::drive_callback, this, std::placeholders::_1));
    // stop_robot = this->create_subscription<std_msgs::msg::Bool>("stop_robot", 10, 
    //                                                             std::bind(&RfidModbusDriver::stop_robot_callback, this, std::placeholders::_1));
    // BatterySub = this->create_subscription<std_msgs::msg::Float32MultiArray>("battery", 10, 
    //                                                             std::bind(&RfidModbusDriver::battery_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&RfidModbusDriver::drive_callback, this));
    // rclcpp::Rate loop_rate(10); // run at 10 Hz
    // battery_percentage = 0;
    // charge_status = 0 ;
    // while (rclcpp::ok())
    // {
    //     RCLCPP_INFO(this->get_logger(), "Publishing:");
    //     publish();
    //     rclcpp::spin_some(node);
    //     loop_rate.sleep();  // wait for the specified rate
    // }
}

void RfidModbusDriver::drive_callback()
{

    // RCLCPP_INFO(get_logger(), "Received Battery Percentage: %d and change status %d", battery_percentage,charge_status);
    modbus_t *ctx = NULL;
    // modbus_t *ctx_plc = NULL;
    // uint16_t tab_reg[5];
    // uint16_t sonar_write_reg[8] = {};
    // uint16_t motor_write_reg[2] = {};
    // uint16_t battery_write_reg[2] = {};
    // uint16_t read_station_reg[2];
    // uint16_t tab_reg1[3];
    int rc;
    uint16_t tab_reg[5];
    // int i;
    // Connect to modbus RTU device
    ctx = modbus_new_rtu("/dev/ttyACM1", 115200, 'N', 8, 1);

    if (ctx == NULL)
    {
     RCLCPP_ERROR(this->get_logger(), "Unable to create the libmodbus context");
     return;
    }

    int status = modbus_connect(ctx);

    if (status == -1)
    {
     RCLCPP_ERROR(this->get_logger(), "RFID Connection failed");
     modbus_free(ctx);
     return;
    }
    rc = modbus_set_slave(ctx, 5);
    rc = modbus_read_input_registers(ctx, 1001, 6, tab_reg);

    if (rc == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read data from Rfid sensor: %s", modbus_strerror(errno));
    }
    else
    {
        
        std_msgs::msg::Int16MultiArray rfid_data;
        rfid_data.data.push_back(tab_reg[0]);
        rfid_data.data.push_back(tab_reg[1]);
        rfid_data.data.push_back((uint16_t) (tab_reg[4] & 0xFF) * std::pow(2,8) + (uint16_t) (tab_reg[5] & 0xFF));
        // data.data.push_back(tab_reg[3]);
        RfidPublisher->publish(rfid_data);
    }
    delay(10);
    modbus_close(ctx);
    modbus_free(ctx);
   
    
   
}

RfidModbusDriver::~RfidModbusDriver()
{

    modbus_close(ctx);
    modbus_free(ctx);
}

void RfidModbusDriver::delay(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
