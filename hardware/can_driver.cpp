#include "kalman_arm_controller/can_driver.hpp"

namespace CAN_driver
{
    int sock = 0;
    struct sockaddr_can addr = {};
    struct ifreq ifr = {};
}

/**
 * @brief Initialize the CAN driver.
 *
 * Initializes the CAN driver and binds the socket to the can0 interface
 *
 * @return int 0 on success, 1 on failure
 */
int CAN_driver::init()
{
    printf("In CAN_driver::init\r\n");
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

    // Set timeout for reading
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

    printf("Finished CAN init! \r\n");
    return 0;
}

/**
 * @brief Read data from the CAN bus.
 *
 * Reads data from the CAN bus one frame by the time.
 * Also handles the received frames by calling `handle_frame` for each frame.
 *
 * @return int 0 on success, 1 on failure
 */
int CAN_driver::read()
{
    // printf("In CAN_driver::read\r\n");
    int nbytes = CANFD_MTU;

    uint8_t *data;

    data = (uint8_t *)malloc(nbytes);

    while (nbytes == CANFD_MTU)
    {
        nbytes = ::read(sock, data, nbytes);
        // printf("Read %d bytes\r\n", nbytes);

        if (nbytes < 0)
        {
            perror("Read");
            return 1;
        }

        struct canfd_frame frame;

        // Parse the data
        frame = *((struct canfd_frame *)data);

        // printf("Received frame with ID %03X and length %d\r\n", frame.can_id, frame.len);
        handle_frame(frame);
    }

    return 0;
}

/**
 * @brief Handle a received frame.
 *
 * Decodes the frame and calls the appropriate handler function.
 *
 * @param frame The frame to handle
 * @return int 0 on success, 1 on failure
 */
int CAN_driver::handle_frame(canfd_frame frame)
{
    // Decode the frame
    uint8_t joint_id = frame.can_id >> 7;
    uint8_t command = frame.can_id - (joint_id << 7);
    try
    {
        printf("Handling frame with ID %03X and length %d, command: %d, joint id:  %d\r\n", frame.can_id, frame.len, command, joint_id);
        if (CAN_handlers::HANDLES.find(command) != CAN_handlers::HANDLES.end())
            CAN_handlers::HANDLES[command].func(frame.can_id, frame.data, frame.len);
    }
    catch (const std::exception &e)
    {
        printf("Caught exception: %s\r\n", e.what());
        return 1;
    }
    return 0;
}

int CAN_driver::write()
{
    // Write data from global joints
    for (int i = 4; i <= 4; i++)
    {
        write_joint_setpoint(i);
        // usleep(1000);
    }

    return 0;
}

int CAN_driver::write_joint_setpoint(uint8_t joint_id)
{
    // Write data from a single joint
    joint_id += 1;

    uint16_t can_id = (joint_id << 7) + CMD_SETPOINT;
    if (1 <= joint_id && joint_id <= 4)
    {
        return write_data(can_id, (uint8_t *)&CAN_vars::joints[joint_id - 1].setpoint, sizeof(jointCmdSetpoint_t));
    }
    else if (joint_id == 5 && joint_id == 6)
    {
        // TODO implement differential
    }
    return 1;
}

int CAN_driver::write_data(uint16_t can_id, uint8_t *data, uint8_t len)
{
    struct canfd_frame frame;
    frame.can_id = can_id;
    frame.len = len;
    memcpy(frame.data, data, len);
    printf("Writing frame with ID %03X and length %d\r\n\tData: {", frame.can_id, frame.len);
    // print data
    for (int i = 0; i < len; i++)
    {
        printf("%02X ", frame.data[i]);
    }
    printf("}\r\n");

    if (::write(sock, &frame, sizeof(frame)) < 0)
    {
        perror("Write");
        return 1;
    }
    return 0;
}

int CAN_driver::close()
{
    return (::close(sock) < 0);
}