#include "follower/Follower.h"
using std::placeholders::_1;
Follower::Follower() : Node("follower")
{
  // Initialize the fork detection variables
  left_value_ = 0;
  middle_value_ = 0;
  right_value_ = 0;
  obstacle_stop_robot_ = false;
  rfid_stop_robot_ =  false;
  follower_state = false;
  follower_direction = true;
  // Subscribe to the Int16MultiArray topic
  sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
    "magnetic_tape_data", 10, std::bind(&Follower::multiArrayCallback, this, std::placeholders::_1));
  rfid_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
    "rfid_tape_data", 10, std::bind(&Follower::rfidArrayCallback, this, std::placeholders::_1));
  lidar_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
    "/zone", 10, std::bind(&Follower::zone_callback, this, std::placeholders::_1));
  

  // service_ = this->create_service<agv_srv::srv::StationSwitch>("enable_line_follower",
  //                                                                  std::bind(&Follower::handle_service, this, std::placeholders::_1, std::placeholders::_2));
  plc_data_sub = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "plc_data", 10, std::bind(&Follower::hardware_callback, this, _1));
  // Create the command velocity publisher
  pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/onix_velocity_controller/cmd_vel_unstamped", 10);
 
}

void Follower::hardware_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "start Program: '%d' Mode %d", msg->data[2],msg->data[3]);
  // RCLCPP_INFO(this->get_logger(), "station %d",msg->data[0]);
  if (msg->data[2] && !msg->data[3]){
    follower_state = true;
  }
  else{
    follower_state = false;
  }
  if (msg->data[4]){
    follower_direction = false; //reverse
  }
  else{
    follower_direction = true; //forward
  }
  // RCLCPP_INFO(this->get_logger(), "state %d",follower_state);
  received_station = msg->data[0];
  // response->status = follower_state;
  // response->success = true;
  // return true;
}




// bool Follower::handle_service(const std::shared_ptr<agv_srv::srv::StationSwitch::Request> request,
//                             std::shared_ptr<agv_srv::srv::StationSwitch::Response> response)
// {
//   RCLCPP_INFO(this->get_logger(), "station %d",request->station);
//   follower_state = request->data;
//   received_station = request->station;
//   response->status = follower_state;
//   // response->success = true;
//   return true;

// }


void  Follower::zone_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
  obstacle_stop_robot_ = msg->data[0];
//  stop_robot_ = msg->data[0];
}

void Follower::rfidArrayCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
 
  // Check that the array has 4 values

  // Extract the fork detection values
  rfid_detection = msg->data[0];
  rfid_signal = msg->data[1];


  // Extract the magnetic tape value
  rfid_data = msg->data[2];
  // RCLCPP_INFO(this->get_logger(), "rfid %d",rfid_data);
  
  if(received_station == rfid_data){
    rfid_stop_robot_ = true;
  }
  else{
    rfid_stop_robot_ = false;
  }
  
  
  
}
void Follower::delay(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void Follower::multiArrayCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
  double speed = 0;
  double direction = 0;
  
  follower_state = 1;
  if (follower_state)
  {
    // Forward
    if(follower_direction){
         // Extract the fork detection values
      left_value_ = msg->data[3];
      middle_value_ = msg->data[4];
      right_value_ = msg->data[5];

      // Extract the magnetic tape value
      int tape_value = msg->data[3];
      RCLCPP_INFO(this->get_logger(), "Forward left  %d Forward middle %d forward right %d",left_value_,middle_value_,right_value_);
      // Compute the proportional control output
      // max +85 to -85
      double error = right_value_ *0.1; // Center the tape value around 0
      // if (tape_value){
        double output = (KP * error)/100.0;
        // RCLCPP_INFO(this->get_logger(), "ERROR %lf, output : %lf",error,output);

        // Calculate the linear velocity using a proportional controller
        // linear_velocity = Kp_linear_ * (1 - std::abs(error/100.0));
        linear_velocity = 0.3;
        // cmd_vel_msg->linear.x = linear_velocity;
        RCLCPP_INFO(this->get_logger(), "output %lf",output);
        // Limit the speed based on the control output
        speed = std::min(MAX_SPEED, std::abs(output));

        // Determine the direction based on the sign of the control output
        direction = (output >= 0) ? 1.0 : -1.0;

      
      
      // Publish the command velocity message
      publishCommandVelocity(speed, direction);
    }

    //Reverse
    else{

      left_value_ = msg->data[0];
      middle_value_ = msg->data[1];
      right_value_ = msg->data[2];

      // Extract the magnetic tape value
      int tape_value = msg->data[3];
      RCLCPP_INFO(this->get_logger(), "Rear left  %d Rear middle %d Rear right %d",left_value_,middle_value_,right_value_);
      // Compute the proportional control output
      // max +85 to -85
      double error = left_value_ *0.1; // Center the tape value around 0
      // if (tape_value){
        double output = (KP * error)/100.0;
        // RCLCPP_INFO(this->get_logger(), "ERROR %lf, output : %lf",error,output);

        // Calculate the linear velocity using a proportional controller
        // linear_velocity = Kp_linear_ * (1 - std::abs(error/100.0));
        linear_velocity = -0.3;
        // cmd_vel_msg->linear.x = linear_velocity;
        RCLCPP_INFO(this->get_logger(), "output %lf",output);
        // Limit the speed based on the control output
        speed = std::min(MAX_SPEED, std::abs(output));

        // Determine the direction based on the sign of the control output
        direction = (output >= 0) ? 1.0 : -1.0;

      
      
      // Publish the command velocity message
      publishCommandVelocity(speed, direction);

      
    }
   
  }

}

void Follower::publishCommandVelocity(double speed, double direction)
{
  // Create the command velocity message
  auto vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
  vel_msg->linear.x = linear_velocity;
  vel_msg->angular.z = speed * direction;
  RCLCPP_INFO(this->get_logger(), "rfid_stop_robot_ %d obstacle_stop_robot_ %d",rfid_stop_robot_,obstacle_stop_robot_);
 if(!rfid_stop_robot_ && !obstacle_stop_robot_){
   // Publish the message
  pub_->publish(std::move(vel_msg));
 }
 
}

