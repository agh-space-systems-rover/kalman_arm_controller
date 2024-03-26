#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_VARS_HPP
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_VARS_HPP

#include "can_types.hpp"
#include "arm_config.hpp"

namespace CAN_vars
{
    extern jointStatus_t joints[6];

    extern armConfig_t arm_config;

    void update_joint_status();
    void update_joint_setpoint();

    void update_single_joint_status(uint8_t joint_id);
    void calculate_status(uint8_t joint_id);
    void calculate_status_diff(uint8_t joint_id, uint8_t diff_id);
    void update_single_joint_setpoint(uint8_t joint_id);
}

#endif // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_VARS_HPP
