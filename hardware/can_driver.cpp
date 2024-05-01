#include "kalman_arm_controller/can_driver.hpp"
#include <poll.h>
#include <cstdio>
#include <mutex>
#include <cerrno>
#include <cstring>
#include "kalman_arm_controller/arm_config.hpp"
#include "kalman_arm_controller/can_handlers.hpp"
#include "kalman_arm_controller/can_vars.hpp"

#define BUFFER_SIZE 1024
#define TIMEOUT_MS 1  // 5 seconds

/**
 * @brief Initialize the CAN driver.
 *
 * Initializes the CAN driver and binds the socket to the can0 interface
 *
 * @return int 0 on success, 1 on failure
 */
int CanDriver::init()
{
  // Get socket connection
  if ((this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("Socket");
    return 1;
  }

  // Enable FD frames
  int enable_fd_frames = 1;
  setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd_frames, sizeof(enable_fd_frames));

  // Set up the can interface
  struct ifreq ifr = {};
  strcpy(ifr.ifr_name, this->can_interface);
  ioctl(sock, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr = {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  // Bind the socket
  if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    perror("Bind");
    return 1;
  }

  // Set timeout for reading
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 1;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

  // ! Export this elsewhere
  this->reader = std::thread(&CanDriver::read, this);

  printf("Finished CAN init! \r\n");
  return 0;
}

int CanDriver::read()
{
  char buffer[BUFFER_SIZE];
  while (should_run)
  {
    ssize_t num_bytes = recv(sock, buffer, BUFFER_SIZE, MSG_DONTWAIT);

    if (num_bytes < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        // there was nothing to read (recv MSG_DONTWAIT flag docs)
        std::this_thread::sleep_for(std::chrono::milliseconds{ 1 });
        continue;
      }
      else
      {
        RCLCPP_FATAL(rclcpp::get_logger("my_logger"), "CanDriver::read: recv failed due to: %s\r\n",
                     std::strerror(errno));
        exit(EXIT_FAILURE);
      }
    }

    buffer[num_bytes] = '\0';  // Null-terminate the received data
    struct canfd_frame frame;

    frame = *((struct canfd_frame*)buffer);

    // We don't need to lock the recv invocation
    std::lock_guard<std::mutex> lock(m_read);  // Yay for RAII
    handle_frame(frame);

    // ! Export this elsewhere
    CAN_vars::update_joint_status();
  }
  return 0;
}

int CanDriver::write_data(uint16_t can_id, uint8_t* data, uint8_t len)
{
  struct canfd_frame frame;
  frame.can_id = can_id;
  frame.len = len;
  frame.flags = 0;
  memcpy(frame.data, data, len);

  if (::write(sock, &frame, sizeof(frame)) < 0)
  {
    perror("Write");
    return 1;
  }
  return 0;
}

int CanDriver::close()
{
  CanDriver::should_run = false;
  CanDriver::reader.join();
  return (::close(sock) < 0);
}

int ArmCanDriver::init()
{
  printf("In ArmCanDriver::init\r\n");
  CanDriver::init();

  // Load default configuration
  arm_config::load_default_config();

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
int ArmCanDriver::handle_frame(canfd_frame frame)
{
  // Decode the frame
  uint8_t joint_id = frame.can_id >> 7;
  uint8_t command = frame.can_id - (joint_id << 7);
  try
  {
    if (CAN_handlers::ARM_HANDLES.find(command) != CAN_handlers::ARM_HANDLES.end())
      CAN_handlers::ARM_HANDLES[command].func(frame.can_id, frame.data, frame.len);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(rclcpp::get_logger("my_logger"), "Caught exception: %s\r\n", e.what());
    return 1;
  }
  return 0;
}

int ArmCanDriver::write(ControlType controlType)
{
  std::lock_guard<std::mutex> lock(CanDriver::m_write);

  CAN_vars::update_joint_setpoint();
  write_control_type(controlType);
  std::this_thread::sleep_for(std::chrono::microseconds(1000));

  // Write data from global joints
  for (int i = 0; i < 6; i++)
  {
    switch (controlType)
    {
      case ControlType::position:
        // printf("position\r\n");
        write_joint_setpoint(i);
        break;

      case ControlType::posvel:
        // printf("posvel\r\n");
        write_joint_posvel(i);
        break;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }

  return 0;
}

int ArmCanDriver::write_control_type(ControlType controlType)
{
  jointCmdControlType_t data;
  switch (controlType)
  {
    case ControlType::position:
      data.controlMode = controlMode_t::CONTROL_MODE_POSITION;
      break;

    case ControlType::posvel:
      data.controlMode = controlMode_t::CONTROL_MODE_LEGACY;
      break;
  }

  uint16_t can_id = CMD_CONTROL_TYPE;
  return write_data(can_id, (uint8_t*)&data, LEN_CMD_CONTROL_TYPE);
}

int ArmCanDriver::write_joint_setpoint(uint8_t joint_id)
{
  // Write data from a single joint
  joint_id += 1;

  uint16_t can_id = (joint_id << 7) + CMD_SETPOINT;
  if (1 <= joint_id && joint_id <= 6)
  {
    return write_data(can_id, (uint8_t*)&CAN_vars::joints[joint_id - 1].setpoint, sizeof(jointCmdSetpoint_t));
  }
  return 1;
}

int ArmCanDriver::write_joint_posvel(uint8_t joint_id)
{
  joint_id += 1;

  uint16_t can_id = (joint_id << 7) + CMD_VELOCITY;
  if (1 <= joint_id && joint_id <= 6)
  {
    return write_data(can_id, (uint8_t*)&CAN_vars::joints[joint_id - 1].velSetpoint, sizeof(jointCmdVelocity_t));
  }
  return 1;
}
