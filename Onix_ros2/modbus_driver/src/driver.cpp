#include "modbus_driver/modbus_driver.h"
#include <cmath>
using namespace std;

ModbusDriver::ModbusDriver() : Node("modbus_publisher")
{
    counter = 0;
    // SonarFrontpublisher = this->create_publisher<std_msgs::msg::Int16MultiArray>("sonar_f_data", 10);
    // SonarRearpublisher = this->create_publisher<std_msgs::msg::Int16MultiArray>("sonar_r_data", 10);
    // MagneticTapePublisher = this->create_publisher<std_msgs::msg::Int16MultiArray>("magnetic_tape_data", 10);
    // RfidPublisher = this->create_publisher<std_msgs::msg::Int16MultiArray>("rfid_tape_data", 10);
    PLCPublisher = this->create_publisher<std_msgs::msg::Int16MultiArray>("plc_data", 10);
    driver_sub = this->create_subscription<onix_msgs::msg::Drive>("/cmd_drive", rclcpp::SensorDataQoS(),
                                                                  std::bind(&ModbusDriver::drive_callback, this, std::placeholders::_1));
    stop_robot = this->create_subscription<std_msgs::msg::Bool>("stop_robot", 10, 
                                                                std::bind(&ModbusDriver::stop_robot_callback, this, std::placeholders::_1));
    
    BatterySub = this->create_subscription<std_msgs::msg::Float32MultiArray>("battery", 10, 
                                                                std::bind(&ModbusDriver::battery_callback, this, std::placeholders::_1));
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ModbusDriver::publish, this));
    // rclcpp::Rate loop_rate(10); // run at 10 Hz
    battery_percentage = 0;
    charge_status = 0 ;
    stop_robot_reg = 0;
    // while (rclcpp::ok())
    // {
    //     // RCLCPP_INFO(this->get_logger(), "Publishing:");
    //     // publish();
    //     // rclcpp::spin_some(node);
    //     // loop_rate.sleep();  // wait for the specified rate
    // }
}

void ModbusDriver::stop_robot_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "stop_robot %d ",msg->data);
    // Handle the received boolean message
  if (msg->data)
  {
     stop_robot_reg = 1;
  }
  else
  {
     stop_robot_reg = 0;
  }
}

void ModbusDriver::battery_callback(const std_msgs::msg::Float32MultiArray::SharedPtr battery_msg){

     if (battery_msg->data.size() > 0)
        {
            battery_percentage = int(battery_msg->data[0]);
            charge_status = int(battery_msg->data[1]);
            // RCLCPP_INFO(get_logger(), "Received Battery Percentage: %d and change status %d", battery_percentage,charge_status);
        }

}


double ModbusDriver::accelerateDecelerate(double previous_value, double target_value,
                                 double acceleration, double deceleration)
    {
        // Apply acceleration and deceleration to reach the target value
        if (target_value > previous_value)
        {
            return std::min(target_value, previous_value + acceleration);
        }
        else if (target_value < previous_value)
        {
            return std::max(target_value, previous_value - deceleration);
        }
        else
        {
            return target_value;
        }
    }

void ModbusDriver::drive_callback(const onix_msgs::msg::Drive::SharedPtr drive_msg)
{

    // int mode = drive_msg->mode;
    double left_wheel = drive_msg->drivers[0];
    double right_wheel = drive_msg->drivers[1];
    double left_wheel_ = 0.0,right_wheel_ = 0.0; 

    

    // Apply acceleration and deceleration to the linear and angular velocities
    left_wheel_ =
        accelerateDecelerate(previous_left_wheel,left_wheel, acceleration, deceleration);
    right_wheel_ =
        accelerateDecelerate(previous_right_wheel, right_wheel, acceleration, deceleration);
    

    // RCLCPP_INFO(this->get_logger(), "left_wheel %lf  right %lf",left_wheel_,right_wheel_);

    // Store the current cmd_vel for the next cycle
    previous_left_wheel = left_wheel_;
    previous_right_wheel = right_wheel_;
    // RCLCPP_INFO(get_logger(), "Received Battery Percentage: %d and change status %d", battery_percentage,charge_status);
    modbus_t *ctx = NULL;
    modbus_t *ctx_plc = NULL;
    // uint16_t tab_reg[5];
    // uint16_t sonar_write_reg[8] = {};
    uint16_t motor_write_reg[2] = {};
    uint16_t battery_write_reg[2] = {};
    uint16_t read_station_reg[2];
    // uint16_t tab_reg1[3];
    int rc;
    // uint16_t tab_reg[5];
    // int i;
    // Connect to modbus RTU device
   
   /*
    ctx = modbus_new_rtu("/dev/ttyACM0", 9600, 'N', 8, 1);

    if (ctx == NULL)
    {
     RCLCPP_ERROR(this->get_logger(), "Unable to create the libmodbus context");
     
    }

    int status = modbus_connect(ctx);

    if (status == -1)
    {
     RCLCPP_ERROR(this->get_logger(), "RFID Connection failed");
     modbus_free(ctx);
     
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
        rfid_data.data.push_back(tab_reg[5]);
        // data.data.push_back(tab_reg[3]);
        RfidPublisher->publish(rfid_data);
    }
    delay(10);
    */
    // ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1);
    // int status1 = modbus_connect(ctx);

    // if (status == -1)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "sonar Connection failed");
    //     modbus_close(ctx);
    //     modbus_free(ctx);
    //     // return;
    // }
    // else
    // {
    //     // sonar front
    //     rc = modbus_set_slave(ctx, 2);

    //     rc = modbus_read_registers(ctx, 262, 4, tab_reg);
        // int a[5] = {0};
        // int c = a[0];

    //     if (rc == -1)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to read data from Sonar front sensor: %s", modbus_strerror(errno));
    //     }
    //     else
    //     {
    //         std_msgs::msg::Int16MultiArray data;
    //         data.data.push_back(tab_reg[0]);
    //         data.data.push_back(tab_reg[1]);
    //         data.data.push_back(tab_reg[2]);
    //         data.data.push_back(tab_reg[3]);
    //         sonar_write_reg[0] = tab_reg[0];
    //         sonar_write_reg[1] = tab_reg[1];
    //         sonar_write_reg[2] = tab_reg[2];
    //         sonar_write_reg[3] = tab_reg[3];
    //         SonarFrontpublisher->publish(data);
    //     }
    //     delay(10);

    //     // sonar rear
    //     rc = modbus_set_slave(ctx, 3);
    //     rc = modbus_read_registers(ctx, 262, 4, tab_reg);

    //     if (rc == -1)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to read data from Sonar rear sensor: %s", modbus_strerror(errno));
            //   sensorstatus = 1;
            
            
    //     }    
    //     else
    //     {
    //         std_msgs::msg::Int16MultiArray data;
    //         data.data.push_back(tab_reg[0]);
    //         data.data.push_back(tab_reg[1]);
    //         data.data.push_back(tab_reg[2]);
    //         data.data.push_back(tab_reg[3]);
    //         sonar_write_reg[4] = tab_reg[0];
    //         sonar_write_reg[5] = tab_reg[1];
    //         sonar_write_reg[6] = tab_reg[2];
    //         sonar_write_reg[7] = tab_reg[3];
    //         SonarRearpublisher->publish(data);
              // sensorstatus = 0;
            
    //     }
    //     delay(10);
    //     modbus_close(ctx);
    //     modbus_free(ctx);
    // }
    
    // PLC connection
    ctx_plc = modbus_new_rtu("/dev/ttyACM0", 115200, 'N', 8, 1);

    int status1 = modbus_connect(ctx_plc);
    // cout << status << endl;
    if (status1 == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "PLC Connection failed");
        modbus_close(ctx_plc);
        modbus_free(ctx_plc);
        return;
    }
    else
    {   double rpm;
        rpm = (left_wheel * 60.0)/(2.0*M_PI);  // converting rad/sec to rpm (rpm = (rad/sec * 60)/2*pi)
        motor_write_reg[0] = rpm /60 * 4000 ; // motor set to 800 ppr . so that if i give 800 ppr and gear ratio
        rpm = (right_wheel * 60.0)/(2.0*M_PI);  // converting rad/sec to rpm (rpm = (rad/sec * 60)/2*pi)
        motor_write_reg[1] = rpm /60 * 4000 ; // motor set to 800 ppr . so that if i give 800 ppr and gear ratio
        // motor_write_reg[0] = left_wheel;
        // motor_write_reg[1] = right_wheel;

        // plc
        rc = modbus_set_slave(ctx_plc, 1);
        // rc = modbus_write_registers(ctx_plc, 4246, 8, sonar_write_reg);
        rc = modbus_write_bit(ctx_plc, 2282, 1);

        // modbus_read_bits(ctx, 2198, 4, read_coil);
        // modbus_read_registers(ctx, 4246, 2, read_station_reg);
        rc = modbus_read_registers(ctx_plc, 4246, 3, read_station_reg);
        // cout <<"\n "<< "Reg Count " <<rc << std::endl;
        // read_station_reg[0]
        // RCLCPP_INFO(get_logger()," station data [%d] [%d]",read_station_reg[0],read_station_reg[1]);
        // RCLCPP_INFO(rclcpp::get_logger(HW_NAME), " coil [%d] [%d] [%d] [%d]", read_coil[0], read_coil[1], read_coil[2], read_coil[3]);
        std_msgs::msg::Int16MultiArray plcdata;
        plcdata.data.push_back(read_station_reg[0]);
        plcdata.data.push_back(read_station_reg[1]);
        plcdata.data.push_back(read_station_reg[2]);
        PLCPublisher->publish(plcdata);

        battery_write_reg[0] = battery_percentage;
        battery_write_reg[1] = charge_status;
        // RCLCPP_INFO(get_logger(), "Received Battery Percentage: %d and change status %d", battery_percentage,charge_status);
        // if (rc == -1)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to write data Plc for motor %s", modbus_strerror(errno));
        // }
        rc = modbus_write_registers(ctx_plc, 4196, 2, motor_write_reg);
        rc = modbus_write_registers(ctx_plc, 4198, 2, battery_write_reg);
        if (stop_robot_reg){
            stop_robot_reg = 0;
            rc = modbus_write_register(ctx_plc, 4246, stop_robot_reg);
            
        } 
        

        if (rc == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write data Plc for motor %s", modbus_strerror(errno));
        }
        modbus_close(ctx_plc);
        modbus_free(ctx_plc);
    }
    
   
}

ModbusDriver::~ModbusDriver()
{

    modbus_close(ctx);
    modbus_free(ctx);
}

void ModbusDriver::delay(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
