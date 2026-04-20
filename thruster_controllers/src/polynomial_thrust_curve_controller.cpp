//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "thruster_controllers/polynomial_thrust_curve_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <ranges>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace thruster_controllers
{

namespace
{

constexpr double KGF_TO_NEWTON = 9.80665;

[[nodiscard]] auto trim(const std::string & s) -> std::string
{
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

}  // namespace

auto PolynomialThrustCurveController::on_init() -> controller_interface::CallbackReturn
{
  param_listener_ = std::make_shared<polynomial_thrust_curve_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  logger_ = get_node()->get_logger();
  return controller_interface::CallbackReturn::SUCCESS;
}

auto PolynomialThrustCurveController::update_parameters() -> void  // NOLINT
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();
}

auto PolynomialThrustCurveController::configure_parameters() -> controller_interface::CallbackReturn
{
  update_parameters();
  thruster_name_ = params_.thruster;

  // Try to load measured lookup table first.
  // Falls back to old polynomial only if you decide to keep that path.
  std::string csv_path;
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("thruster_controllers");
    csv_path = share_dir + "/t200_measured_data/pwm_thrust_measurements.csv";

  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to locate thruster_controllers share directory: %s", e.what());  // NOLINT
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!load_lookup_table_from_csv(csv_path)) {
    RCLCPP_ERROR(logger_, "Failed to load thrust/PWM lookup table from: %s", csv_path.c_str());  // NOLINT
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Loaded %zu thrust/PWM samples from %s", thrust_pwm_table_.size(), csv_path.c_str());  // NOLINT
  return controller_interface::CallbackReturn::SUCCESS;
}

auto PolynomialThrustCurveController::load_lookup_table_from_csv(const std::string & csv_path) -> bool
{
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Could not open CSV file: %s", csv_path.c_str());  // NOLINT
    return false;
  }

  thrust_pwm_table_.clear();

  std::string line;
  if (!std::getline(file, line)) {
    RCLCPP_ERROR(logger_, "CSV file is empty: %s", csv_path.c_str());  // NOLINT
    return false;
  }

  // Expect header like:
  // PWM (µs),Force (Kg f)
  while (std::getline(file, line)) {
    line = trim(line);
    if (line.empty()) {
      continue;
    }

    std::stringstream ss(line);
    std::string pwm_str;
    std::string thrust_str;

    if (!std::getline(ss, pwm_str, ',')) {
      continue;
    }
    if (!std::getline(ss, thrust_str, ',')) {
      continue;
    }

    try {
      const double pwm = std::stod(trim(pwm_str));
      const double thrust_kgf = std::stod(trim(thrust_str));
      const double thrust_newtons = thrust_kgf * KGF_TO_NEWTON;

      thrust_pwm_table_.push_back(ThrustPwmSample{
        .thrust = thrust_newtons,
        .pwm = pwm,
      });
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "Skipping malformed CSV row '%s': %s", line.c_str(), e.what());  // NOLINT
    }
  }

  if (thrust_pwm_table_.size() < 2) {
    RCLCPP_ERROR(logger_, "Lookup table must contain at least 2 valid rows");  // NOLINT
    return false;
  }

  std::sort(
    thrust_pwm_table_.begin(), thrust_pwm_table_.end(),
    [](const ThrustPwmSample & a, const ThrustPwmSample & b) {
      return a.thrust < b.thrust;
    });

  return true;
}

auto PolynomialThrustCurveController::lookup_pwm_from_thrust(double thrust_newtons) const -> int
{
  if (thrust_pwm_table_.empty()) {
    return params_.neutral_pwm;
  }

  // Clamp below measured range
  if (thrust_newtons <= thrust_pwm_table_.front().thrust) {
    return static_cast<int>(std::round(thrust_pwm_table_.front().pwm));
  }

  // Clamp above measured range
  if (thrust_newtons >= thrust_pwm_table_.back().thrust) {
    return static_cast<int>(std::round(thrust_pwm_table_.back().pwm));
  }

  const auto upper = std::lower_bound(
    thrust_pwm_table_.begin(), thrust_pwm_table_.end(), thrust_newtons,
    [](const ThrustPwmSample & sample, double value) {
      return sample.thrust < value;
    });

  const auto lower = std::prev(upper);

  const double t0 = lower->thrust;
  const double t1 = upper->thrust;
  const double p0 = lower->pwm;
  const double p1 = upper->pwm;

  if (std::abs(t1 - t0) < 1e-9) {
    return static_cast<int>(std::round(p0));
  }

  const double alpha = (thrust_newtons - t0) / (t1 - t0);
  const double pwm = p0 + alpha * (p1 - p0);

  return static_cast<int>(std::round(pwm));
}

auto PolynomialThrustCurveController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  if (configure_parameters() != controller_interface::CallbackReturn::SUCCESS) {
    return controller_interface::CallbackReturn::ERROR;
  }

  reference_.writeFromNonRT(std_msgs::msg::Float64());
  command_interfaces_.reserve(1);

  reference_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/reference", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<std_msgs::msg::Float64> msg) {  // NOLINT
      reference_.writeFromNonRT(*msg);
    });

  controller_state_pub_ = get_node()->create_publisher<ControllerState>("~/status", rclcpp::SystemDefaultsQoS());
  rt_controller_state_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<ControllerState>>(controller_state_pub_);

  controller_state_.dof_state.name = thruster_name_;

  return controller_interface::CallbackReturn::SUCCESS;
}

auto PolynomialThrustCurveController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  -> controller_interface::CallbackReturn
{
  reference_.readFromNonRT()->data = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_.assign(reference_interfaces_.size(), std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

auto PolynomialThrustCurveController::command_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration command_interface_config;
  command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interface_config.names.reserve(1);
  command_interface_config.names.emplace_back(std::format("{}/pwm", thruster_name_));
  return command_interface_config;
}

auto PolynomialThrustCurveController::state_interface_configuration() const
  -> controller_interface::InterfaceConfiguration
{
  controller_interface::InterfaceConfiguration state_interface_config;
  state_interface_config.type = controller_interface::interface_configuration_type::NONE;
  return state_interface_config;
}

auto PolynomialThrustCurveController::on_export_reference_interfaces()
  -> std::vector<hardware_interface::CommandInterface>
{
  reference_interfaces_.resize(1, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(reference_interfaces_.size());

  interfaces.emplace_back(
    get_node()->get_name(),
    std::format("{}/{}", thruster_name_, hardware_interface::HW_IF_EFFORT),
    &reference_interfaces_[0]);  // NOLINT(readability-container-data-pointer)

  return interfaces;
}

auto PolynomialThrustCurveController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/) -> controller_interface::return_type
{
  auto * current_reference = reference_.readFromNonRT();
  reference_interfaces_[0] = current_reference->data;
  current_reference->data = std::numeric_limits<double>::quiet_NaN();
  return controller_interface::return_type::OK;
}

auto PolynomialThrustCurveController::update_and_write_commands(
  const rclcpp::Time & time,
  const rclcpp::Duration & period) -> controller_interface::return_type
{
  const auto reference = reference_interfaces_[0];
  int pwm = params_.neutral_pwm;

  if (!std::isnan(reference)) {
    const double clamped_reference = std::clamp(reference, params_.min_thrust, params_.max_thrust);
    pwm = lookup_pwm_from_thrust(clamped_reference);
    pwm = pwm > params_.min_deadband_pwm && pwm < params_.max_deadband_pwm ? params_.neutral_pwm : pwm;
  }

  if (!command_interfaces_[0].set_value(static_cast<double>(pwm))) {
    RCLCPP_WARN(logger_, "Failed to set command for thruster %s", thruster_name_.c_str());  // NOLINT
  }

  const auto out = command_interfaces_[0].get_optional();
  controller_state_.header.stamp = time;
  controller_state_.dof_state.reference = reference_interfaces_[0];
  controller_state_.dof_state.time_step = period.seconds();
  controller_state_.dof_state.output = out.value_or(std::numeric_limits<double>::quiet_NaN());
  rt_controller_state_pub_->try_publish(controller_state_);

  return controller_interface::return_type::OK;
}

}  // namespace thruster_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  thruster_controllers::PolynomialThrustCurveController,
  controller_interface::ChainableControllerInterface)