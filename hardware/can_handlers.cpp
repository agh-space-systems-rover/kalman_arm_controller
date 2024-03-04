#include "can_handlers.hpp"

int CAN_handlers::handle_joint_status(uint32_t identifier, uint8_t *data, uint8_t len)
{
    uint8_t joint_id = identifier >> 7;
    uint8_t command = identifier - (joint_id << 7);

    jointMotorStatus_t *status = (jointMotorStatus_t *)data;

    // Update the joint status
    CAN_driver::joints[joint_id].status = *status;
    return 0;
}