#include "kalman_arm_controller/can_lib/can_types.hpp"
#include "kalman_arm_controller/can_lib/master_handlers.hpp"
#include "kalman_arm_controller/can_lib/master_messages.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <memory>

class CanMaster
{
private:
  static std::shared_ptr<rclcpp::Node> node;
  static std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Joy_<std::allocator<void> >, std::allocator<void> > >
      joy_pub;

public:
  static int init()
  {
    node = rclcpp::Node::make_shared("can_master");
    joy_pub = node->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    return 0;
  }
  static int handle_joy(uint32_t identifier, uint8_t* data, uint8_t len)
  {
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = node->now();
    joy_msg.header.frame_id = "joy";
    joy_msg.axes.resize(6, 0);
    joy_msg.buttons.resize(12, 0);

    for (int i = 0; i < 6; i++)
    {
      // TODO calculate the correct scaling factor
      joy_msg.axes[i] = ((float*)data)[i];
    }

    joy_pub->publish(joy_msg);
    return 0;
  }
};

namespace CAN_handlers
{
// Define the command handler array
std::unordered_map<uint8_t, canCmdHandler_t> MASTER_HANDLES = {
  { CMD_JOY_CONTROL, { CMD_JOY_CONTROL, sizeof(joyControl_t), CanMaster::handle_joy } },
  //   { CMD_JOINT_FAST_STATUS, { CMD_JOINT_FAST_STATUS, sizeof(jointMotorFastStatus_t), handle_joint_fast_status } }
};

}  // namespace CAN_handlers
