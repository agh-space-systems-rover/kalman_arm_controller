#include "kalman_arm_controller/can_driver.hpp"
#include <poll.h>
#include <mutex>
#include <vector>

#define BUFFER_SIZE 1024
#define TIMEOUT_MS 1 // 5 seconds

namespace CAN_driver
{
    int sock = 0;
    struct sockaddr_can addr = {};
    struct ifreq ifr = {};
    std::mutex m;
    std::thread reader;
    bool should_run = true;
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
    strcpy(ifr.ifr_name, "can1");
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

    CAN_driver::reader = std::thread(CAN_driver::read);

    printf("Finished CAN init! \r\n");
    return 0;
}

int CAN_driver::read()
{
    char buffer[BUFFER_SIZE];
    bool should_sleep = false;
    while (CAN_driver::should_run){
        if (should_sleep){
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        should_sleep = false;
        std::lock_guard<std::mutex> lock(CAN_driver::m); // Yay for RAII

        ssize_t num_bytes = recv(sock, buffer, BUFFER_SIZE, MSG_DONTWAIT);
        if (num_bytes < 0)
        {
            should_sleep = true;
            continue;
        }

        buffer[num_bytes] = '\0'; // Null-terminate the received data
        struct canfd_frame frame;

        frame = *((struct canfd_frame *)buffer);
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
        // RCLCPP_INFO(rclcpp::get_logger("my_logger"), "Handling frame with ID %03X and length %d, command: %d, joint id:  %d\r\n", frame.can_id, frame.len, command, joint_id);
        if (CAN_handlers::HANDLES.find(command) != CAN_handlers::HANDLES.end())
            CAN_handlers::HANDLES[command].func(frame.can_id, frame.data, frame.len);
    }
    catch (const std::exception &e)
    {
        // RCLCPP_INFO(rclcpp::get_logger("my_logger"), "Caught exception: %s\r\n", e.what());
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
    CAN_driver::should_run = false;
    CAN_driver::reader.join();
    return (::close(sock) < 0);
}