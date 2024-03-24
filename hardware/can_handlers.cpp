#include "kalman_arm_controller/can_handlers.hpp"

#include <stdio.h>

namespace CAN_handlers
{
    // Define the command handler array
    std::unordered_map<uint8_t, canCmdHandler_t> HANDLES = {
        {CMD_JOINT_STATUS, {CMD_JOINT_STATUS, sizeof(jointMotorStatus_t), handle_joint_status}},
    };

    /**
     * @brief Handle the joint status command.
     *
     * Updates the global joint status.
     */
    int handle_joint_status(uint32_t identifier, uint8_t *data, uint8_t len)
    {
        uint8_t joint_id = identifier >> 7;
        uint8_t command = identifier - (joint_id << 7);

        joint_id--;

        jointMotorStatus_t *status = (jointMotorStatus_t *)data;
        // TODO: Handle differential joints
        // Update the joint status
        CAN_vars::joints[joint_id].status = *status;
        printf("Joint %d:\r\n\tVelocity: %d\r\n\tPosition: %d\r\n", joint_id, CAN_vars::joints[joint_id].status.velocity, CAN_vars::joints[joint_id].status.position);

        return 0;
    }

} // namespace CAN_handlers