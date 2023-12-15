

#include "onix_hardware/onix_hardware_interface.hpp"

using onix_hardware::OnixHardwareInterface;

/**
 * @brief Construct a new OnixHardwareInterface object
 * 
 */
OnixHardwareInterface::OnixHardwareInterface()
: Node("onix_hardware")
{
  feedback_sub_ = create_subscription<onix_msgs::msg::Feedback>(
    "feedback",
    rclcpp::SensorDataQoS(),
    std::bind(&OnixHardwareInterface::feedback_callback, this, std::placeholders::_1));

  drive_pub_ = create_publisher<onix_msgs::msg::Drive>(
    "cmd_drive",
    rclcpp::SensorDataQoS());
}

/**
 * @brief Feedback subscription callback
 * 
 * @param msg 
 */
void OnixHardwareInterface::feedback_callback(const onix_msgs::msg::Feedback::SharedPtr msg)
{
  std::lock_guard<std::mutex> guard(feedback_mutex_);
  feedback_ = *msg;
}

/**
 * @brief Publish Drive message
 * 
 * @param left_wheel Left wheel command
 * @param right_wheel Right wheel command
 * @param mode Command mode
 */
void OnixHardwareInterface::drive_command(float left_wheel, float right_wheel, int8_t mode)
{
  onix_msgs::msg::Drive drive_msg;
  drive_msg.mode = mode;
  drive_msg.drivers[onix_msgs::msg::Drive::LEFT] = left_wheel;
  drive_msg.drivers[onix_msgs::msg::Drive::RIGHT] = right_wheel;
  drive_pub_->publish(drive_msg);
}

/**
 * @brief Get latest feedback message
 * 
 * @return onix_msgs::msg::Feedback message 
 */
onix_msgs::msg::Feedback OnixHardwareInterface::get_feedback()
{
  onix_msgs::msg::Feedback msg;

  {
    std::lock_guard<std::mutex> guard(feedback_mutex_);
    msg = feedback_;
  }

  return msg;
}
