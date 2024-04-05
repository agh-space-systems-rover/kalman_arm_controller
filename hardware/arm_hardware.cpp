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

        // CAN_driver::init();

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
        for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
        {
            joint_velocities_[i] = joint_velocities_command_[i];
            joint_position_[i] += joint_velocities_command_[i] * period.seconds();
        }

        for (auto i = 0ul; i < joint_position_command_.size(); i++)
        {
            joint_position_[i] = joint_position_command_[i];
        }
        return return_type::OK;
        // return read_joint_states();
    }

    return_type ArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return return_type::OK;
        // return write_joint_commands();
    }

    return_type ArmSystem::read_joint_states()
    {
        // CAN_driver::read();
        std::lock_guard<std::mutex> lock(CAN_driver::m_read);
        RCLCPP_INFO(rclcpp::get_logger("my_logger"), "position: %ld", CAN_vars::joints[0].status.position);

        for (int i = 0; i < 6; i++)
        {
            joint_position_[i] = CAN_vars::joints[i].moveStatus.position_deg;
            joint_velocities_[i] = CAN_vars::joints[i].moveStatus.velocity_deg_s;
        }
        return return_type::OK;
    }

    return_type ArmSystem::write_joint_commands()
    {
        {
            std::lock_guard<std::mutex> lock(CAN_driver::m_write);
            for (int i = 0; i < 4; i++)
            {
                CAN_vars::joints[i].moveSetpoint.position_deg = joint_position_command_[i];
                CAN_vars::joints[i].moveSetpoint.velocity_deg_s = joint_velocities_command_[i];
                CAN_vars::joints[i].moveSetpoint.torque_Nm = 0x02fa;
                CAN_vars::joints[i].moveSetpoint.acceleration_deg_ss = 0xffff;
            }
            for (int i = 4; i < 6; i++)
            {
                CAN_vars::joints[i].moveSetpointDiff.position_deg = joint_position_command_[i];
                CAN_vars::joints[i].moveSetpointDiff.velocity_deg_s = joint_velocities_command_[i];
                CAN_vars::joints[i].moveSetpointDiff.torque_Nm = 0x02fa;
                CAN_vars::joints[i].moveSetpointDiff.acceleration_deg_ss = 0xffff;
            }
        }

        // Do not write if previous write is still in progress
        if (writer.valid() && writer.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        {
            RCLCPP_WARN(rclcpp::get_logger("my_logger"), "Previous write still in progress");
        }
        else
        {
            // Run write in a separate thread
            writer = std::async(std::launch::async, [&]
                                { CAN_driver::write(); });
        }

        return return_type::OK;
    }

} // namespace kalman_arm_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    kalman_arm_controller::ArmSystem, hardware_interface::SystemInterface)
