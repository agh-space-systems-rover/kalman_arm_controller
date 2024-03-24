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
    uint8_t data[LEN_CMD_VELOCITY] = {0xfa, 0x02, 0xff, 0xff, 0xd9, 0x09};
    for (int i = 0; i < 10; i++)
    {
        CAN_driver::write_data(can_id, data, LEN_CMD_VELOCITY);
        usleep(1000);
    }

    can_id = (4 << 7) + CMD_SETPOINT;
    uint8_t data_pos[LEN_CMD_SETPOINT] = {0xfa, 0x02, 0xff, 0xff, 0x27, 0xf6, 0xa0, 0xc9, 0xfa, 0xff, 0xcd, 0x00};

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for (int i = 0; i < 100; i++)
    {
        CAN_driver::write_data(can_id, data_pos, LEN_CMD_SETPOINT);
        usleep(10000);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        printf("Writing took %ld [us]\r\n", std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count());

        begin = std::chrono::steady_clock::now();
    }

    uint8_t data_pos2[LEN_CMD_SETPOINT] = {0xfa, 0x02, 0xff, 0xff, 0x00, 0x00, 0xa0, 0xc9, 0xfa, 0xff, 0xcd, 0x00};

    for (int i = 0; i < 100; i++)
    {
        // CAN_driver::write_data(can_id, data_pos2, LEN_CMD_SETPOINT);
        usleep(10000);
    }

    int8_t setpoint_cnt = 0;

    while (1)
    {

        for (int i = 0; i < 6; i++)
        {
            if (i == 3)
            {
                CAN_vars::joints[i].setpoint.torque_mNm = 0x02fa;
                CAN_vars::joints[i].setpoint.acceleration_0RPMs_1 = 0xffff;
                // CAN_vars::joints[i].setpoint.velocity_0RPM_1 = (setpoint_cnt % 2 ? -1 : 1) * 2521;
                // CAN_vars::joints[i].setpoint.velocity_0RPM_1 = 2521;
                CAN_vars::joints[i].setpoint.velocity_0RPM_1 = setpoint_cnt * 20;
                CAN_vars::joints[i].setpoint.position_0deg01 = -setpoint_cnt * 1000;
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
        printf("Setting joints setpoints\r\n");

        setpoint_cnt++;
        printf("Writing CAN data\r\n");
        try
        {
            if (CAN_driver::write())
                break;
        }
        catch (const std::exception &e)
        {
            printf("Caught exception: %s\r\n", e.what());
            CAN_driver::close();
            return 1;
        }
        printf("Written!\r\n\r\n");
        // usleep(10000);
    }
    CAN_driver::close();
    return 0;
}