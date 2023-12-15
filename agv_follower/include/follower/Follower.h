#ifndef FOLLOWER_HPP_
#define FOLLOWER_HPP_

// #include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "agv_srv/srv/station_switch.hpp"
#include <ctime>
#include <unistd.h>

class Follower : public rclcpp::Node
{
public:
  Follower();

private:
  void delay(int milliseconds);
  void multiArrayCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
  void stateArrayCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
  void zone_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
  void publishCommandVelocity(double speed, double direction);
  bool handle_service(const std::shared_ptr<agv_srv::srv::StationSwitch::Request> request,
                      std::shared_ptr<agv_srv::srv::StationSwitch::Response> response);
  void hardware_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
  bool follower_state,follower_direction;
  // Constants for proportional following
  double KP = 0.552;
  double MAX_SPEED = 0.7;
  double Kp_linear_ = 1.33;
  double linear_velocity;

  // Global variables for fork detection
  int left_value_;
  int middle_value_;
  int right_value_;
  int last_value_;
  int last_value = 0;
  int direction_state_,direction_speed_,direction_value_,direction_control_;
  double linear_speed_,ang_kp_;
  int rfid_detection,received_station,rfid_signal,rfid_data;
  double robot_speed,robot_kp;     //include robot kp and speed
  bool obstacle_stop_robot_,rfid_stop_robot_;
  std::string path_values,path_way;
  //int state = 0;
  int tape_avg_;
  double time_delay_;
  bool time_switch_ = false;
  bool line_switch_ ;
  bool line_switch2=false;
  bool line_switch3=false;
  int start_time_;
  double linear_error_,linear_error_kp_,linear_speed_error_;

  // ROS publishers and subscribers
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr robot_state_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr  lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  // rclcpp::Service<agv_srv::srv::StationSwitch>::SharedPtr service_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr plc_data_sub;
};

#endif  // FOLLOWER_HPP_
