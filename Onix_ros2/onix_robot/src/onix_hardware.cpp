
#include "onix_hardware/onix_hardware.hpp"
#include "onix_msgs/msg/feedback.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace onix_hardware
{

static const std::string HW_NAME = "OnixHardware";
static const std::string LEFT_CMD_JOINT_NAME = "front_left_wheel_joint";
static const std::string RIGHT_CMD_JOINT_NAME = "front_right_wheel_joint";
static const std::string LEFT_ALT_JOINT_NAME = "rear_left_wheel_joint";
static const std::string RIGHT_ALT_JOINT_NAME = "rear_right_wheel_joint";

/**
 * @brief Write commanded velocities to the MCU
 * 
 */
void OnixHardware::writeCommandsToHardware()
{
  //  double diff_speed_left = hw_commands_[wheel_joints_[LEFT_CMD_JOINT_NAME]];
  // double diff_speed_right = hw_commands_[wheel_joints_[RIGHT_CMD_JOINT_NAME]];
  double diff_speed_left = hw_commands_[0];
  double diff_speed_right = hw_commands_[1];

  if (std::abs(diff_speed_left) < 0.01 && std::abs(diff_speed_right) < 0.01) {
    diff_speed_left = diff_speed_right = 0.0;
  }

  node_->drive_command(
    static_cast<float>(diff_speed_left),
    static_cast<float>(diff_speed_right),
    onix_msgs::msg::Drive::MODE_VELOCITY);
}

/**
 * @brief Pull latest speed and travel measurements from MCU, 
 * and store in joint structure for ros_control
 * 
 */
void OnixHardware::updateJointsFromHardware()
{
  rclcpp::spin_some(node_);
  onix_msgs::msg::Feedback msg = node_->get_feedback();
  RCLCPP_DEBUG(
    rclcpp::get_logger(HW_NAME),
    "Received linear distance information (L: %f, R: %f)",
    msg.drivers[0].measured_travel, msg.drivers[1].measured_travel);

  auto side = onix_msgs::msg::Drive::LEFT;
  for (auto i = 0u; i < hw_states_position_.size(); i++) {
    if (i == wheel_joints_[RIGHT_ALT_JOINT_NAME] || i == wheel_joints_[RIGHT_CMD_JOINT_NAME]){
      side = onix_msgs::msg::Drive::RIGHT;
    }
    else {
      side = onix_msgs::msg::Drive::LEFT;
    }

    double delta = msg.drivers[side].measured_travel -
      hw_states_position_[i] - hw_states_position_offset_[i];

    // detect suspiciously large readings, possibly from encoder rollover
    if (std::abs(delta) < 1.0f) {
      hw_states_position_[i] += delta;
    } else {
      // suspicious! drop this measurement and update the offset for subsequent readings
      hw_states_position_offset_[i] += delta;
      RCLCPP_WARN(
        rclcpp::get_logger(HW_NAME), "Dropping overflow measurement from encoder");
    }

    // Velocities
    hw_states_velocity_[i] = msg.drivers[side].measured_velocity;
  }
}

hardware_interface::return_type OnixHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Name: %s", info_.name.c_str());

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Number of Joints %u", info_.joints.size());

  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_position_offset_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  node_ = std::make_shared<OnixHardwareInterface>();

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // OnixHardware has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' has %d state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> OnixHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OnixHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));

    // Map wheel joint name to index
    wheel_joints_[info_.joints[i].name] = i;
  }

  return command_interfaces;
}

hardware_interface::return_type OnixHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Starting ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_states_position_.size(); i++) {
    if (std::isnan(hw_states_position_[i])) {
      hw_states_position_[i] = 0;
      hw_states_position_offset_[i] = 0;
      hw_states_velocity_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OnixHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Stopping ...please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OnixHardware::read()
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Reading from hardware");

  updateJointsFromHardware();

  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OnixHardware::write()
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Writing to hardware");

  writeCommandsToHardware();

  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace onix_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  onix_hardware::OnixHardware, hardware_interface::SystemInterface)
