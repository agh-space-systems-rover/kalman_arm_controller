/**
 * @file can_types.hpp
 * @brief Contains the definitions of CAN types used in the Kalman Arm Controller hardware.
 */

#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_

#include <cstdint>
#include <can_messages.hpp>

/**
 * @brief Enumeration representing the positioning status.
 */
typedef enum
{
    POSITIONING_NO,          ///< No positioning
    POSITIONING_IN_PROGRESS, ///< Positioning in progress
    POSITIONING_TIMEOUT,     ///< Positioning timeout
    POSITIONING_ERROR,       ///< Positioning error
    POSITIONING_SUCCESS,     ///< Positioning success
    POSITIONING_ENUM_SIZE    ///< Size of the positioning status enum
} positioningStatus_t;

/**
 * @brief Structure representing a CAN message handler.
 *
 * This structure is used to store the CAN message handler functions.
 *
 * @param can_id uint16_t Command identifier
 * @param len uint8_t Length of the data
 * @param func Function pointer to the handler
 */
typedef struct
{
    uint16_t can_id;
    uint8_t len;
    int (*func)(uint32_t identifier, uint8_t *data, uint8_t len);
} canCmdHandler_t;

/**
 * @brief Structure representing the status of a joint motor and its setpoint.
 *
 * This structure combines the joint motor status and the joint setpoint.
 *
 * @param status jointMotorStatus_t Received joint motor status
 * @param setpoint jointCmdSetpoint_t Joint setpoint to send
 */
typedef struct __attribute__((__packed__))
{
    jointMotorStatus_t status;
    jointCmdSetpoint_t setpoint;
} jointStatus_t;

#endif // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_