

#ifndef ONIX_HARDWARE_ONIX_HARDWARE_INTERFACE_H
#define ONIX_HARDWARE_ONIX_HARDWARE_INTERFACE_H

#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "onix_msgs/msg/drive.hpp"
#include "onix_msgs/msg/feedback.hpp"

namespace onix_hardware
{

class OnixHardwareInterface
: public rclcpp::Node
{
  public:
  explicit OnixHardwareInterface();
  void drive_command(float left_wheel, float right_wheel, int8_t mode);
  onix_msgs::msg::Feedback get_feedback();

  private:
  void feedback_callback(const onix_msgs::msg::Feedback::SharedPtr msg);

  rclcpp::Publisher<onix_msgs::msg::Drive>::SharedPtr drive_pub_;
  rclcpp::Subscription<onix_msgs::msg::Feedback>::SharedPtr feedback_sub_;

  onix_msgs::msg::Feedback feedback_;
  std::mutex feedback_mutex_;
};

}

#endif  // ONIX_HARDWARE_ONIX_HARDWARE_INTERFACE_H