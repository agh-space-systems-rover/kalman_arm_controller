#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_HANDLERS_HPP
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_HANDLERS_HPP

#include "can_types.hpp"
#include "can_driver.hpp"
#include "can_messages.hpp"

namespace CAN_handlers
{
    int handle_joint_status(uint32_t identifier, uint8_t *data, uint8_t len);

    const canCmdHandler_t HANDLES[] = {
        {CMD_JOINT_STATUS, sizeof(jointMotorStatus_t), handle_joint_status},
    };
} // namespace CAN_handlers

#endif // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_HANDLERS_HPP