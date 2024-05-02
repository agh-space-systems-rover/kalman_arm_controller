#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__MASTER_MESSAGES_HPP
#define KALMAN_ARM_CONTROLLER__HARDWARE__MASTER_MESSAGES_HPP
#include <stdint.h>

/*********RECEIVED MESSAGES***********/

/**
 * @brief Structure representing the joystick control message received from the CAN bus.
 *
 * This structure is used to represent the joystick control message received from the CAN bus.
 *
 * @param axes int8_t[6] Axes values - bytes 0-5
 */
typedef struct __attribute__((__packed__))
{
  int8_t axes[6];
} joyControl_t;
#define CMD_JOY_CONTROL 0x036
#define LEN_JOY_CONTROL 6

/*********SENDING MESSAGES************/

/**
 * @brief Structure representing the setpoint to be sent to a joint motor through the CAN bus.
 *
 * This structure is used to send the setpoint to the joint motor through the CAN bus.
 *
 * @param position_0deg01 int32_t Position in 0.01 degrees units - bytes 0-3
 */
// typedef struct __attribute__((__packed__))
// {
//   int32_t position_0deg01;
// } jointCmdSetpoint_t;
// #define CMD_SETPOINT 0x026
// #define LEN_CMD_SETPOINT 4

#endif  // KALMAN_ARM_CONTROLLER__HARDWARE__MASTER_MESSAGES_HPP
