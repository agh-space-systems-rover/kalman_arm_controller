#ifndef KALMAN_ARM_CONTROLLER__HARDWARE__CAN_DRIVER_HPP
#define KALMAN_ARM_CONTROLLER__HARDWARE__CAN_DRIVER_HPP

#include "can_types.hpp"
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <sys/socket.h>
#include <netinet/in.h>
#include "rclcpp/rclcpp.hpp"

/*
Potrzebne:
- 6 jointów globalnych
- konfiguracja CAN - wysłąnie ramki velocity
- odczytywanie ramek, dekodowanie i zapisanie stanu do odpowiedniego jointa
- wysyłanie ramek z wartościami zadanych do odpowiednich jointów
*/

class CanDriver
{
protected:
  int sock = 0;
  const char* can_interface = "can1";
  std::unordered_map<uint8_t, canCmdHandler_t>* handles;

  int handle_frame(canfd_frame frame);
  int write_data(uint16_t can_id, uint8_t* data, uint8_t len);
  int read(char* buffer);

public:
  std::mutex m_read;
  std::mutex m_write;
  std::thread reader;
  bool should_run = true;

  CanDriver(const char* can_interface) : can_interface(can_interface)
  {
  }
  int init();
  virtual int read_loop();
  int close();
};

class ArmCanDriver : public CanDriver
{
private:
  int write_control_type(ControlType controlType);
  int write_joint_setpoint(uint8_t joint_id);
  int write_joint_posvel(uint8_t joint_id);

public:
  ArmCanDriver(const char* can_interface) : CanDriver(can_interface)
  {
  }
  int init();
  int read_loop();
  int write(ControlType controlType);
};

class MasterCanDriver : public CanDriver
{
public:
  MasterCanDriver(const char* can_interface) : CanDriver(can_interface)
  {
  }
  int init();
  int read_loop();
  int write();
};

#endif  // KALMAN_ARM_CONTROLLER__HARDWARE__CAN_DRIVER_HPP