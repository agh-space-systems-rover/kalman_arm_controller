#include "can_handlers.hpp"

namespace CAN_handlers
{
    /**
     * @brief Handle the joint status command.
     *
     * Updates the global joint status.
     */
    int handle_joint_status(uint32_t identifier, uint8_t *data, uint8_t len)
    {
        uint8_t joint_id = identifier >> 7;
        uint8_t command = identifier - (joint_id << 7);

        jointMotorStatus_t *status = (jointMotorStatus_t *)data;
        // TODO: Handle differential joints
        // Update the joint status
        CAN_driver::joints[joint_id].status = *status;
        return 0;
    }

} // namespace CAN_handlers