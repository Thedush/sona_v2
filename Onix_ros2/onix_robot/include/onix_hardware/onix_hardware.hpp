

#ifndef ONIX_HARDWARE_ONIX_HARDWARE_H
#define ONIX_HARDWARE_ONIX_HARDWARE_H

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

#include "onix_hardware_interface.hpp"


namespace onix_hardware
{

class OnixHardware
  : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OnixHardware)

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:
  void writeCommandsToHardware();
  void updateJointsFromHardware();

  std::shared_ptr<OnixHardwareInterface> node_;

  // Store the command for the robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_, hw_states_position_offset_, hw_states_velocity_;

  std::map<std::string, uint8_t> wheel_joints_;
};

}  // namespace onix_hardware

#endif  // ONIX_HARDWARE_ONIX_HARDWARE_H
