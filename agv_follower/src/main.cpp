#include "follower/Follower.h"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char *argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<Follower>();

  // Spin the node
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
