#include "kalman_arm_controller/can_driver.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <exception>

int main()
{
    printf("Testing can_driver.cpp\r\n");
    CAN_driver::init();
    while (1)
    {
        printf("Reading CAN data\r\n");
        try
        {
            if (CAN_driver::read())
                continue;
        }
        catch (const std::exception &e)
        {
            printf("Caught exception: %s\r\n", e.what());
            CAN_driver::close();
            return 1;
        }
        printf("Done reading CAN data\r\n");
        printf("Active joints status:\r\n");
        for (int i = 0; i < 6; i++)
        {
            printf("Joint %d:\r\n\tVelocity: %d\r\n\tPosition: %d\r\n", i, CAN_vars::joints[i].status.velocity, CAN_vars::joints[i].status.position);
        }
        printf("\r\n");
        usleep(1000000);
    }
    CAN_driver::close();
    return 0;
}
