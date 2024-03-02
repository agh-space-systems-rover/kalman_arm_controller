#include "kalman_arm_controller/can_driver.hpp"

namespace CAN_driver
{
    int sock;
    struct sockaddr_can addr;
    struct ifreq ifr;
    jointStatus_t joints[6];


    bool init()
    {
        // Get socket connection
        if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            perror("Socket");
            return 1;
        }

        // Enable FD frames
        int enable_fd_frames = 1;
        setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd_frames, sizeof(enable_fd_frames));

        // Set up the can interface
        strcpy(ifr.ifr_name, "can0");
        ioctl(sock, SIOCGIFINDEX, &ifr);

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        // Bind the socket
        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            perror("Bind");
            return 1;
        }

        return 0;
    }

    bool read()
    {
        struct canfd_frame frame;
        int nbytes;

        // TODO: fionread to read all available frames
        nbytes = read(sock, &frame, sizeof(struct canfd_frame));

        // TODO: scan all bytes and update joint status
        if (nbytes < 0)
        {
            perror("Read");
            return 1;
        }
    }

    bool write()
    {
        struct canfd_frame frame;



    }

    bool close()
    {
        return (::close(sock) < 0);
    }
}