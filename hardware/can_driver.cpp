#include "kalman_arm_controller/can_driver.hpp"

namespace CAN_driver
{
    int init()
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

    int read()
    {
        int nbytes = 0;

        uint8_t *data;

        // Get available bytes
        ioctl(sock, FIONREAD, &nbytes);

        data = (uint8_t *)malloc(nbytes);

        nbytes = ::read(sock, data, nbytes);

        if (nbytes < 0)
        {
            perror("Read");
            return 1;
        }

        struct canfd_frame frame;

        // Parse the data
        for (int i = 0; i < nbytes; i += CANFD_MTU)
        {
            frame = *((struct canfd_frame *)data + i);

            if (frame.len > 0)
            {
                // Handle the frame
                handle_frame(frame);
            }
        }
    }

    int handle_frame(canfd_frame frame)
    {
        // Decode the frame
        uint8_t joint_id = frame.can_id >> 7;
        uint8_t command = frame.can_id - (joint_id << 7);

        CAN_handlers::HANDLES[command].func(frame.can_id, frame.data, frame.len);
    }

    int write()
    {
        struct canfd_frame frame;
    }

    int close()
    {
        return (::close(sock) < 0);
    }
}