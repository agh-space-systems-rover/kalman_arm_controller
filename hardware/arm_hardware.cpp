#include "kalman_arm_controller/arm_hardware.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "arm_hardware.hpp"

namespace kalman_arm_controller
{
    CallbackReturn ArmSystem::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // robot has 6 joints and 2 interfaces
        joint_position_.assign(6, 0);
        joint_velocities_.assign(6, 0);
        joint_position_command_.assign(6, 0);
        joint_velocities_command_.assign(6, 0);

        for (const auto &joint : info_.joints)
        {
            for (const auto &interface : joint.state_interfaces)
            {
                joint_interfaces[interface.name].push_back(joint.name);
            }
        }

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ArmSystem::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        
        int ind = 0;
        for (const auto &joint_name : joint_interfaces["position"])
        {
            state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
        }

        ind = 0;
        for (const auto &joint_name : joint_interfaces["velocity"])
        {
            state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ArmSystem::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        int ind = 0;
        for (const auto &joint_name : joint_interfaces["position"])
        {
            command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
        }

        ind = 0;
        for (const auto &joint_name : joint_interfaces["velocity"])
        {
            command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
        }

        return command_interfaces;
    }

    return_type ArmSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        return read_joint_states();
    }

    return_type ArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return write_joint_commands();
    }

    return_type ArmSystem::read_joint_states()
    {
        CAN_driver::read();
        // TODO read config and data from CAN_vars::joints and update joint_position_ and joint_velocities_
        return return_type::OK;
    }

    return_type ArmSystem::write_joint_commands()
    {
        for (int i = 0; i < 6; i++)
        {
            CAN_vars::joints[i].setpoint.position_0deg01 = int32_t(joint_position_[i]*0.01);
            // joint_velocities_command_[i] = joint_velocities_[i];
            // CAN_driver::write_joint_setpoint(i);
        }
        return return_type::OK;

        // TODO write commands to CAN_vars::joints from joint_position_command_ and joint_velocities_command_
    }

} // namespace kalman_arm_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    kalman_arm_controller::ArmSystem, hardware_interface::SystemInterface)
