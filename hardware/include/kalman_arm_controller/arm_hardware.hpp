#ifndef KALMAN_ARM_CONTROLLER__ARM_HARDWARE_HPP_
#define KALMAN_ARM_CONTROLLER__ARM_HARDWARE_HPP_

#include <string>
#include <unordered_map>
#include <vector>
#include <future>

#include "can_driver.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;

namespace kalman_arm_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC ArmSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;

protected:
  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;

  std::future<void> writer;

  ArmCanDriver can_driver = ArmCanDriver("can1");

  ControlType current_control_type = ControlType::posvel;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = { { "position", {} },
                                                                                 { "velocity", {} } };

  return_type read_joint_states();
  return_type write_joint_commands();
};

}  // namespace kalman_arm_controller

#endif  // KALMAN_ARM_CONTROLLER__ARM_HARDWARE_HPP_