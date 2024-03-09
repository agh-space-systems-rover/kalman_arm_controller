#include "../hardware/include/kalman_arm_controller/can_driver.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <exception>

int main()
{
    printf("Testing can_driver.cpp\r\n");
    CAN_driver::init();

    // Write velocity command to enable the joints
    uint16_t can_id = (1 << 7) + CMD_VELOCITY;
    uint8_t data[LEN_CMD_VELOCITY] = {0xff, 0xff, 0xff, 0xff, 0, 0};
    CAN_driver::write_data(can_id, data, LEN_CMD_VELOCITY);

    uint8_t setpoint_cnt = 0;
    while (1)
    {
        printf("Setting joints setpoints\r\n");
        for (int i = 0; i < 6; i++)
        {
            if (i == setpoint_cnt % 6)
            {
                CAN_vars::joints[i].setpoint.torque_mNm = 0xfa02;
                CAN_vars::joints[i].setpoint.acceleration_0RPMs_1 = 0xffff;
                CAN_vars::joints[i].setpoint.velocity_0RPM_1 = (setpoint_cnt / 6 % 2 ? -1 : 1) * 9975;
                CAN_vars::joints[i].setpoint.position_0deg01 = 1000;
                CAN_vars::joints[i].setpoint.lastSetpointTime = setpoint_cnt;
            }
            else
            {
                CAN_vars::joints[i].setpoint.torque_mNm = 0xfa02;
                CAN_vars::joints[i].setpoint.acceleration_0RPMs_1 = 0xffff;
                CAN_vars::joints[i].setpoint.velocity_0RPM_1 = 0;
                CAN_vars::joints[i].setpoint.position_0deg01 = 1000;
                CAN_vars::joints[i].setpoint.lastSetpointTime = setpoint_cnt;
            }
        }
        setpoint_cnt++;
        printf("Writing CAN data\r\n");
        try
        {
            CAN_driver::write();
        }
        catch (const std::exception &e)
        {
            printf("Caught exception: %s\r\n", e.what());
            CAN_driver::close();
            return 1;
        }
        printf("Written!\r\n\r\n");
        usleep(1000000);
    }
    CAN_driver::close();
    return 0;
}