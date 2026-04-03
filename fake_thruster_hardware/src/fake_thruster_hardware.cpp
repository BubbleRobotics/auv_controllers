#include "fake_thruster_hardware/fake_thruster_hardware.hpp"

#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace fake_thruster_hardware
{

hardware_interface::CallbackReturn FakeThrusterHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(logger_, "Failed to initialize FakeThrusterHardware");
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  joint_names_.reserve(info_.joints.size());

  for (const auto & joint : info_.joints) {
    const bool has_pwm_cmd_interface =
      std::any_of(
        joint.command_interfaces.begin(),
        joint.command_interfaces.end(),
        [](const auto & iface) { return iface.name == "pwm"; });

    if (!has_pwm_cmd_interface) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s' does not expose required command interface 'pwm'",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    joint_names_.push_back(joint.name);
  }

  hw_commands_.assign(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_.assign(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(
    logger_,
    "Initialized FakeThrusterHardware with %zu joints",
    joint_names_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeThrusterHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  for (auto & cmd : hw_commands_) {
    cmd = std::numeric_limits<double>::quiet_NaN();
  }
  for (auto & state : hw_states_) {
    state = std::numeric_limits<double>::quiet_NaN();
  }

  RCLCPP_INFO(logger_, "Configured FakeThrusterHardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeThrusterHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (auto & cmd : hw_commands_) {
    cmd = 1500.0;
  }
  for (auto & state : hw_states_) {
    state = 1500.0;
  }

  RCLCPP_INFO(logger_, "Activated FakeThrusterHardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeThrusterHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivated FakeThrusterHardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
FakeThrusterHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_states_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FakeThrusterHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(
      joint_names_[i],
      "pwm",
      &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type FakeThrusterHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // Mirror the current command into a dummy state so there is something defined.
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_states_[i] = hw_commands_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FakeThrusterHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // Intentionally do nothing.
  // Your external node can observe /thruster_i_controller/status.output
  // and drive Gazebo directly.
  return hardware_interface::return_type::OK;
}

}  // namespace fake_thruster_hardware

PLUGINLIB_EXPORT_CLASS(
  fake_thruster_hardware::FakeThrusterHardware,
  hardware_interface::SystemInterface)