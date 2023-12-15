#include "follower/Follower.h"
using std::placeholders::_1;
Follower::Follower() : Node("follower")
{
  // Initialize the fork detection variables
  left_value_ = 0;
  middle_value_ = 0;
  right_value_ = 0;
  obstacle_stop_robot_ = false;
  rfid_stop_robot_ = false;
  follower_state = false;
  follower_direction = true;
  line_switch2 = false;
  line_switch3 = false;
  direction_state_ = 0;
  direction_speed_ = 0;
  direction_control_ = 0;
  direction_value_ = 0;
  linear_speed_ = 0;
  ang_kp_ = 0;
  time_delay_ = 0;
  start_time_ = 0;
  linear_error_ = 0;
  linear_error_kp_ = 0;
  linear_speed_error_ = 0;
  // tape_avg_=0;

  // Subscribe to the Int16MultiArray topic
  sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "color_line", 10, std::bind(&Follower::multiArrayCallback, this, std::placeholders::_1));
  robot_state_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "state", 10, std::bind(&Follower::stateArrayCallback, this, std::placeholders::_1));
  lidar_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/zone", 10, std::bind(&Follower::zone_callback, this, std::placeholders::_1));

  // service_ = this->create_service<agv_srv::srv::StationSwitch>("enable_line_follower",
  //                                                                  std::bind(&Follower::handle_service, this, std::placeholders::_1, std::placeholders::_2));
  plc_data_sub = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "plc_data", 10, std::bind(&Follower::hardware_callback, this, _1));
  // Create the command velocity publisher
  pub_ = this->create_publisher<geometry_msgs::msg::Twist>("nav_vel", 10);
}

void Follower::hardware_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "start Program: '%d' Mode %d", msg->data[2],msg->data[3]);
  // RCLCPP_INFO(this->get_logger(), "station %d",msg->data[0]);
  if (msg->data[2] && !msg->data[3])
  {
    follower_state = true;
  }
  else
  {
    follower_state = false;
  }
  if (msg->data[4])
  {
    follower_direction = false; // reverse
  }
  else
  {
    follower_direction = true; // forward
  }
  // RCLCPP_INFO(this->get_logger(), "state %d",follower_state);
  received_station = msg->data[0];
  // response->status = follower_state;
  // response->success = true;
  // return true;
}

void Follower::zone_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
  obstacle_stop_robot_ = msg->data[0];
  //  stop_robot_ = msg->data[0];g
}
// statecallback
void Follower::stateArrayCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{

  // Check that the array has 4 values
  direction_state_ = msg->data[0];          //from 0 to 2
  // direction_state_ = direction_state_ * 1;
  // Extract the fork detection values
  // linear_velocity = msg->data[1];
  direction_speed_ = msg->data[1];          //from 1 to 2
  // direction_speed_ = direction_speed_ * 1;
  direction_control_ = msg->data[2];  
  RCLCPP_INFO(this->get_logger(), "direction state %d direction speed %d direction control %d  ", direction_state_, direction_speed_,direction_control_);      //for printrfid

  // RCLCPP_INFO(this->get_logger(), "state  %d direction %d",msg->data[0],msg->data[1]);
}
void Follower::delay(int milliseconds)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void Follower::multiArrayCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
  double speed = 0;
  double direction = 0;
  // tape_avg_=((left_value_+right_value_+middle_value_)/3);
  // middle_value_ = msg->data[4];
  // left_value_ = msg->data[0];     //middle actual      //right actual2
  // middle_value_ = msg->data[1];    //right actual     //middle actul2
  // right_value_ = msg->data[2];     //left   actual

  // actual directions
  // left_value_ = msg->data[2];
  middle_value_ = msg->data[0];
  // right_value_ = msg->data[0];
  // time_delay_= 3;

  // Direction Control by RFID
  if (direction_control_ == 0)
  {
    line_switch_ = false;
    line_switch2 = false;
    line_switch3 = false;
    // direction value by RFID

    if (direction_state_ >= 1)
    {

      direction_value_ = middle_value_;
      // RCLCPP_INFO(this->get_logger(), "Robot in middle value"); // takes middle value
    }
    // else if (direction_state_ == 2)
    // {
    //   direction_value_ = left_value_;
    //   // RCLCPP_INFO(this->get_logger(), "Robot in left value"); // takes left value
    // }
    // else if (direction_state_ == 3)
    // {
    //   direction_value_ = right_value_;
    //   // RCLCPP_INFO(this->get_logger(), "Robot in right value"); // takes right value
    // }

    // direction speed by RFID

    if (direction_speed_ == 0)
    {
      ang_kp_ = 0.66;
      linear_error_kp_ = 0.66;
      linear_speed_ = 0.2;
      // time_delay_=6;

    }
    else if (direction_speed_ == 1)
    {
      ang_kp_ = 0.133;
      linear_error_kp_ = 1.33;
      linear_speed_ = 0.4;
      // time_delay_=1;
    }
    else if (direction_speed_ == 2)
    {
      ang_kp_ = 0.233;
      linear_error_kp_ = 2.33;
      linear_speed_ = 0.7;
      // time_delay_=2;
    }

    // Direction State by RFID

    if (direction_state_ == 0)
    {
      follower_state = 0;
      RCLCPP_INFO(this->get_logger(), "direction state %d direction speed %d direction control %d  ", direction_state_, direction_speed_,direction_control_);      //for printrfid
      RCLCPP_INFO(this->get_logger(), "Robot is stopped direction_state %d middle_value %d  ", direction_state_, middle_value_);
    }

    else if (middle_value_ == -32768)
    {
      if (!time_switch_)
      {
        start_time_ = time(0);
        time_switch_ = true;
        last_value = last_value_;
      }
      RCLCPP_INFO(this->get_logger(), "time switch %d  last_value %d ", time_switch_, last_value);
      if ((time(0) - start_time_) < time_delay_)
      {
        follower_state = 1;
        direction_value_ = last_value_;
        // linear_speed_ = 0;
        // RCLCPP_INFO(this->get_logger(), "Robot is turned in optimising ");
      }
      else
      {
        follower_state = 0;
        // RCLCPP_INFO(this->get_logger(), "last value of direction  %d ",direction_value_);
        // RCLCPP_INFO(this->get_logger(), "tape middle value %d ",middle_value_);
      }
    }
    else
    {
      // RCLCPP_INFO(this->get_logger(), "Robot is turned on");
      // RCLCPP_INFO(this->get_logger(), "Robot's middle value %d ", middle_value_); //for checking sensor data
      follower_state = 1;
      time_switch_ = false;
    }

    follower_direction = 0;
    if (follower_state)
    {
      // Forward
      if (follower_direction)
      {
        // Extract the fork detection values
        // left_value_ = msg->data[3];
        // middle_value_ = msg->data[4];
        // right_value_ = msg->data[5];

        // Extract the magnetic tape value
        int tape_value = msg->data[0];
        RCLCPP_INFO(this->get_logger(), "Forward left  %d Forward middle %d forward right %d", left_value_, middle_value_, right_value_);
        // Compute the proportional control output
        // max +85 to -85
        double error = right_value_ * 0.1; // Center the tape value around 0
                                           // if (tape_value){
        double output = (KP * error) / 100.0;
        // RCLCPP_INFO(this->get_logger(), "ERROR %lf, output : %lf",error,output);

        // Calculate the linear velocity using a proportional controller
        // linear_velocity = Kp_linear_ * (1 - std::abs(error/100.0));
        //linear_velocity = 0.6;  //maked comm
        // cmd_vel_msg->linear.x = linear_velocity;
        RCLCPP_INFO(this->get_logger(), "output %lf", output);
        // Limit the speed based on the control output
        speed = std::min(MAX_SPEED, std::abs(output));

        // Determine the direction based on the sign of the control output
        direction = (output >= 0) ? 1.0 : -1.0;

        // Publish the command velocity message
        publishCommandVelocity(speed, direction);
      }

      // Reverse 
      else
      {
        // if (state == 0)

        // Time Delay

        if (direction_value_ > -150 && direction_value_ < 150)  //changed to 150
        {
          time_delay_ = 1;
          RCLCPP_INFO(this->get_logger(), "Time delay 1 %lf", time_delay_);
        }
        else
        {
          time_delay_ = 6;
          RCLCPP_INFO(this->get_logger(), "Time delay 2 %lf", time_delay_);
        }

        // Extract the magnetic tape value
        // int tape_value = msg->data[0];
        RCLCPP_INFO(this->get_logger(), "Rear left  %d Rear middle %d Rear right %d", left_value_, middle_value_, right_value_);
        // Compute the proportional control output
        // max +85 to -85
        double error = direction_value_ * 0.1; // Center the tape value around 0 // to 0.01

        RCLCPP_INFO(this->get_logger(), "Robot in taken value %d  direction_way %d ", direction_value_, direction_state_);
        // if (tape_value){
        double output = (ang_kp_ * error) / 100.0;
        // RCLCPP_INFO(this->get_logger(), " error output %lf", output);
        // RCLCPP_INFO(this->get_logger(), "ERROR %lf, output : %lf",error,output);

        // Calculate the linear velocity using a proportional controller
        // linear_velocity = Kp_linear_ * (1 - std::abs(error/100.0));

        linear_error_ = (300 - (std::abs(direction_value_))) * 0.001; // -0.85 to +0.85 change from 0.01 t0 0.1

        linear_speed_error_ = (linear_error_kp_ * linear_error_);

        linear_velocity = linear_speed_error_; // Proportional linear velocity
        RCLCPP_INFO(this->get_logger(), "linear velocity %lf", linear_velocity);

        // RCLCPP_INFO(this->get_logger(), "robot speed %lf", linear_speed_);
        // cmd_vel_msg->linear.x = linear_velocity;
        RCLCPP_INFO(this->get_logger(), "output %lf", output);
        // Limit the speed based on the control output
        speed = std::min(MAX_SPEED, std::abs(output));
        linear_velocity = std::min(linear_speed_, linear_velocity);

        // Determine the direction based on the sign of the control output
        direction = (output >= 0) ? -1.0 : 1.0; //changed from 1.0 to -1.0

        // Publish the command velocity message
        publishCommandVelocity(speed, direction);

        RCLCPP_INFO(this->get_logger(), "publishing command  speed %lf direction %lf", speed, direction);
      }
    }
    else
    {

      RCLCPP_INFO(this->get_logger(), "Exit from loop ");
    }
    last_value_ = direction_value_;
  }
  /*

  */

  else if (direction_control_ == 1)
  {
    if (middle_value_ != -32768 && !line_switch2)
    {
      RCLCPP_INFO(this->get_logger(), "robot control stage 2 ", middle_value_); //
      linear_velocity = 0;
      speed = 0.2;
      direction = -1;
      publishCommandVelocity(speed, direction);
      line_switch_ = true;
    }
    else
    {
      if ((line_switch_ == true) && (middle_value_ == -32768))
      {
        linear_velocity = 0;
        speed = 0.2;
        direction = -1;
        publishCommandVelocity(speed, direction);
        line_switch2 = true;
        RCLCPP_INFO(this->get_logger(), "publish complted control stage 23  %d", middle_value_);
      }
      else
      {
        // stop the robot
        linear_velocity = 0;
        speed = 0.0;
        direction = 0;
        publishCommandVelocity(speed, direction);
        RCLCPP_INFO(this->get_logger(), "publish complted control stage 3 ");
      }
    }
  }
  else if (direction_control_ == 2)
  {
    if (middle_value_ != -32768 && !line_switch3)
    {
      RCLCPP_INFO(this->get_logger(), "robot control stage 92 ", middle_value_); //
      linear_velocity = 0;
      speed = 0.2;
      direction = 1;
      publishCommandVelocity(speed, direction);
      line_switch_ = true;
    }
    else
    {
      if ((line_switch_ == true) && (middle_value_ == -32768))
      {
        linear_velocity = 0;
        speed = 0.2;
        direction = 1;
        publishCommandVelocity(speed, direction);
        line_switch3 = true;
        RCLCPP_INFO(this->get_logger(), "publish complted control stage 923  %d", middle_value_);
      }
      else
      {
        // stop the robot
        linear_velocity = 0;
        speed = 0.0;
        direction = 0;
        publishCommandVelocity(speed, direction);
        RCLCPP_INFO(this->get_logger(), "publish complted control stage 93 ");
      }
    }
  }
}

void Follower::publishCommandVelocity(double speed, double direction)
{
  // Create the command velocity message
  auto vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
  vel_msg->linear.x = linear_velocity;
  vel_msg->angular.z = speed * direction;
  // RCLCPP_INFO(this->get_logger(), "rfid_stop_robot_ %d obstacle_stop_robot_ %d", rfid_stop_robot_, obstacle_stop_robot_);

  if (!rfid_stop_robot_ && !obstacle_stop_robot_)
  {
    RCLCPP_INFO(this->get_logger(), "linear_x %lf angular_z %lf", linear_velocity, speed * direction);
    // Publish the message
    pub_->publish(std::move(vel_msg));

    // RCLCPP_INFO(this->get_logger(), "sensor mid value %d " ,middle_value_);

    // tape_avg_=((left_value_+right_value_+middle_value_)/3);
  }
}
