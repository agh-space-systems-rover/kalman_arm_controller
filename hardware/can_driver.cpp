#include "kalman_arm_controller/can_driver.hpp"

namespace CAN_driver
{
    /**
     * @brief Initialize the CAN driver.
     *
     * Initializes the CAN driver and binds the socket to the can0 interface
     *
     * @return int 0 on success, 1 on failure
     */
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

    /**
     * @brief Read data from the CAN bus.
     *
     * Reads data from the CAN bus by checking the number of available bytes and then reading all of them.
     * Also handles the received frames by calling `handle_frame` for each frame.
     *
     * @return int 0 on success, 1 on failure
     */
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

    /**
     * @brief Handle a received frame.
     *
     * Decodes the frame and calls the appropriate handler function.
     *
     * @param frame The frame to handle
     * @return int 0 on success, 1 on failure
     */
    int handle_frame(canfd_frame frame)
    {
        // Decode the frame
        uint8_t joint_id = frame.can_id >> 7;
        uint8_t command = frame.can_id - (joint_id << 7);
        try
        {
            CAN_handlers::HANDLES[command].func(frame.can_id, frame.data, frame.len);
        }
        catch (const std::exception &e)
        {
            printf("Caught exception: %s\r\n", e.what());
            return 1;
        }
        return 0;
    }

    int write()
    {
        // Write data from global joints
        for (int i = 1; i <= 6; i++)
        {
            write_joint_setpoint(i);
        }

        return 0;
    }

    int write_joint_setpoint(uint8_t joint_id)
    {
        // Write data from a single joint
        uint16_t can_id = (joint_id << 7) + CMD_SETPOINT;
        if (1 <= joint_id && joint_id <= 4)
        {
            return write_data(can_id, (uint8_t *)&joints[joint_id - 1].setpoint, sizeof(jointCmdSetpoint_t));
        }
        else if (joint_id == 5 && joint_id == 6)
        {
            // TODO implement differential
        }
    }

    int write_data(uint16_t can_id, uint8_t *data, uint8_t len)
    {
        struct canfd_frame frame;
        frame.can_id = can_id;
        frame.len = len;
        memcpy(frame.data, data, len);

        if (::write(sock, &frame, sizeof(frame)) < 0)
        {
            perror("Write");
            return 1;
        }
        return 0;
    }

    int close()
    {
        return (::close(sock) < 0);
    }
}