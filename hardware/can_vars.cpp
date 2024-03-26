#include "kalman_arm_controller/can_vars.hpp"
#include "can_vars.hpp"

namespace CAN_vars
{
    jointStatus_t joints[6] = {};

    armConfig_t arm_config = {0};
}

/**
 * @brief Update joint setpoint from global joints - converts from CAN
 * message to human readable units.
 */
void CAN_vars::update_joint_status()
{
    for (uint8_t i = 0; i < arm_config.jointNumber; i++)
    {
        update_single_joint_status(i);
    }
}

/**
 * @brief Update joint setpoint from global joints depending on differential joint
 *
 * @param joint_id - from 0 to 5
 */
void CAN_vars::update_single_joint_status(uint8_t joint_id)
{
    jointConfig_t *config = &CAN_vars::arm_config.joint[joint_id + 1];
    if (config->differential)
    {
        calculate_status_diff(joint_id, config->differential);
    }
    else
    {
        calculate_status(joint_id);
    }
}

/**
 * @brief Update joint setpoint from global joints - converting to human readable units
 *
 * @param joint_id - from 0 to 5
 */
void CAN_vars::calculate_status(uint8_t joint_id)
{
    jointConfig_t *config = &CAN_vars::arm_config.joint[joint_id + 1];
    jointMotorStatus_t *jointStatus = &CAN_vars::joints[joint_id].status;
    float gearRatio = config->gearRatio;
    int direction = config->invertDirection ? -1 : 1;

    joints[joint_id].moveStatus.torque_Nm = 0.001f * gearRatio * jointStatus->torque * direction;
    joints[joint_id].moveStatus.velocity_deg_s = (360.0f / (10 * 60)) * gearRatio * jointStatus->velocity * direction;
    joints[joint_id].moveStatus.position_deg = 0.01f * gearRatio * jointStatus->position * direction;
}

/**
 * @brief Calculate differential joint status -  into human readable units
 *
 * @param joint_id - from 0 to 5
 * @param diff_id - differential joint id (from 1 to 6)
 */
void CAN_vars::calculate_status_diff(uint8_t joint_id, uint8_t diff_id)
{
    uint8_t difNbr[2];
    jointConfig_t *difConfig[2];
    jointMotorStatus_t *difStatus[2];

    float gearRatio[2], torque[2], velocity[2], position[2];

    if (diff_id > joint_id + 1)
    {
        difNbr[0] = joint_id + 1;
        difNbr[1] = diff_id;
    }
    else
    {
        difNbr[0] = diff_id;
        difNbr[1] = joint_id + 1;
    }

    difConfig[0] = &CAN_vars::arm_config.joint[difNbr[0]];
    difConfig[1] = &CAN_vars::arm_config.joint[difNbr[1]];

    difStatus[0] = &CAN_vars::joints[difNbr[0]].status;
    difStatus[1] = &CAN_vars::joints[difNbr[1]].status;

    for (uint8_t i = 0; i < 2; i++)
    {
        gearRatio[i] = difConfig[i]->gearRatio;
        torque[i] = 0.001f * gearRatio[i] * difStatus[i]->torque;
        velocity[i] = (360.0f / (10 * 60)) * gearRatio[i] * difStatus[i]->velocity;
        position[i] = 0.01f * gearRatio[i] * difStatus[i]->position;
    }

    joints[difNbr[0] - 1].moveStatus.torque_Nm = (torque[0] - torque[1]) / 2;
    joints[difNbr[0] - 1].moveStatus.velocity_deg_s = (velocity[0] - velocity[1]) / 2;
    joints[difNbr[0] - 1].moveStatus.position_deg = (position[0] - position[1]) / 2;

    joints[difNbr[1] - 1].moveStatus.torque_Nm = (torque[0] + torque[1]) / 2;
    joints[difNbr[1] - 1].moveStatus.velocity_deg_s = (velocity[0] + velocity[1]) / 2;
    joints[difNbr[1] - 1].moveStatus.position_deg = (position[0] + position[1]) / 2;
}