#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_

#include <cstdint>

/**
Positioning status enum.
*/
typedef enum
{
    POSITIONING_NO,
    POSITIONING_IN_PROGRESS,
    POSITIONING_TIMEOUT,
    POSITIONING_ERROR,
    POSITIONING_SUCCESS,
    POSITIONING_ENUM_SIZE
} positioningStatus_t;

/**
Joint motor status struct. This struct is used to store the status of the joint motor received from the CAN bus.

Byte 0 - fault:
    - `motorTemperatureFault`: 1 bit
    - `controllerTemperatureFault`: 1 bit
    - `encoderFault`: 1 bit
    - `timeout`: 1 bit
    - `positioningError`: 1 bit
    - `VinToLow`: 1 bit
    - `reserved`: 2 bits

Byte 1:
    - `outputEnable`: 1 bit
    - `commandsBlocked`: 1 bit
    - `motorInitialized`: 1 bit
    - `positioningStatus`: 3 bits
    - `switchState`: 2 bits

Byte 2:
    - `motorTemperature_1deg`: 8 bits (int8_t)

Byte 3:
    - `controllerTemerature_1deg`: 8 bits (int8_t)

Byte 4-5:
    - `torque`: 16 bits (int16_t)

Byte 6-7:
    - `velocity`: 16 bits (int16_t)

Byte 8-11:
    - `position`: 32 bits (int32_t)

Byte 12:
    - `inputVoltage_0V2`: 8 bits (uint8_t)

Byte 13-15:
    - `reserved`: 24 bits - not used
*/
typedef struct __attribute__((__packed__))
{
    union
    {
        struct __attribute__((__packed__))
        {
            uint8_t motorTemperatureFault : 1;
            uint8_t controllerTemperatureFault : 1;
            uint8_t encoderFault : 1;
            uint8_t timeout : 1;
            uint8_t positioningError : 1;
            uint8_t VinToLow : 1;
            uint8_t reserved : 2;
        } bit;

        uint8_t allFault;
    } fault;

    uint8_t outputEnable : 1;
    uint8_t commandsBlocked : 1;
    uint8_t motorInitialized : 1;
    positioningStatus_t positioningStatus : 3;
    uint8_t switchState : 2;

    int8_t motorTemperature_1deg;
    int8_t controllerTemerature_1deg;

    int16_t torque;
    int16_t velocity;
    int32_t position;

    uint8_t inputVoltage_0V2;

    uint8_t reserved[3];
} jointMotorStatus_t;

/**
Joint set point struct. This struct is used to send the setpoint to the joint motor through the CAN bus.

Byte 0-1:
    - `torque_mNm`: 16 bits (uint16_t)

Byte 2-3:
    - `acceleration_0RPMs_1`: 16 bits (uint16_t)

Byte 4-5:
    - `velocity_0RPM_1`: 16 bits (int16_t)

Byte 6-9:
    - `position_0deg01`: 32 bits (int32_t)

Byte 10:
    - `lastSetpointTime`: 8 bits (uint8_t)

Byte 11:
    - `reserved`: 8 bits (uint8_t)
*/
typedef struct __attribute__((__packed__))
{
    uint16_t torque_mNm;
    uint16_t acceleration_0RPMs_1;
    int16_t velocity_0RPM_1;
    int32_t position_0deg01;
    uint8_t lastSetpointTime;
    uint8_t reserved;
} jointCmdSetpoint_t;
#define CMD_SETPOINT 0x026

/**
Joint status struct. Combines the joint motor status and the joint set point.

Parameters:
    - `status`: received joint motor status
    - `setpoint`: joint setpoint to send
*/
typedef struct __attribute__((__packed__))
{
    jointMotorStatus_t status;
    jointCmdSetpoint_t setpoint;
} jointStatus_t;

/**
Joint control velocity struct. This struct is used to send the velocity to the joint motor through the CAN bus.

Byte 0-1:
    - `torque_mNm`: 16 bits (uint16_t)

Byte 2-3:
    - `acceleration_0RPMs_1`: 16 bits (uint16_t)

Byte 4-5:
    - `velocity_0RPM_1`: 16 bits (int16_t)
*/
typedef struct __attribute__((__packed__))
{
    uint16_t torque_mNm;
    uint16_t acceleration_0RPMs_1;
    int16_t velocity_0RPM_1;
} jointCmdVelocity_t;
#define CMD_VELOCITY 0x025

#endif // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_TYPES_HPP_