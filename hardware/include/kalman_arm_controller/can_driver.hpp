#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_DRIVER_HPP
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_DRIVER_HPP

#include "can_types.hpp"
#include "can_handlers.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
/*
Potrzebne:
- 6 jointów globalnych
- konfiguracja CAN - wysłąnie ramki velocity
- odczytywanie ramek, dekodowanie i zapisanie stanu do odpowiedniego jointa
- wysyłanie ramek z wartościami zadanych do odpowiednich jointów
*/

namespace CAN_driver
{
    int sock;
    struct sockaddr_can addr;
    struct ifreq ifr;
    extern jointStatus_t joints[6];

    int init();
    int read();
    int write();
    int handle_frame(canfd_frame frame);
    int close();
}

#endif // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_DRIVER_HPP