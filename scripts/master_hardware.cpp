#include <rclcpp/rclcpp.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/event_handler.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/timer.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <thread>
extern "C" {
#include "kalman_arm_controller/can_libs/can_driver.hpp"
}
#include "kalman_arm_controller/can_libs/can_types.hpp"
#include <map>
#include <future>
#include <chrono>
#include <stdint.h>

const uint16_t WRITE_CALLBACK_PERIOD_US = 10;
const uint16_t READ_CALLBACK_PERIOD_US = 10;
const double JOINT_TIMEOUT = 0.5;

const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";

namespace kalman_master
{

struct
{
  std::vector<uint8_t> velocities;
  rclcpp::Time lastReceivedTime;
} jointsVelData = {};

class CanMasterNode : public rclcpp::Node
{
private:
  CAN_driver::DriverVars_t master_driver_;
  std::future<void> writer;
  rclcpp::TimerBase::SharedPtr write_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

  std::unordered_map<uint8_t, canCmdHandler_t> MASTER_HANDLES = {
    //   { CMD_JOINT_STATUS, { CMD_JOINT_STATUS, sizeof(jointMotorStatus_t), handle_joint_status } },
    //   { CMD_JOINT_FAST_STATUS, { CMD_JOINT_FAST_STATUS, sizeof(jointMotorFastStatus_t), handle_joint_fast_status } }
  };

  void writeCan()
  {
    if (writer.valid() && writer.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      // RCLCPP_WARN(rclcpp::get_logger("my_logger"), "Previous write still in progress");
    }
    else
    {
      {
        std::lock_guard<std::mutex> lock(this->master_driver_.m_write);
        // Do something related to variables that write uses
      }
      // Run write in a separate thread
      writer = std::async(std::launch::async, [&] { CAN_driver::master_write(&master_driver_); });
    }
  }

  void readCan()
  {
    std::lock_guard<std::mutex> lock(this->master_driver_.m_read);
    // Do something related to variables that read uses
    if ((now() - jointsVelData.lastReceivedTime).seconds() > JOINT_TIMEOUT)
    {
      RCLCPP_WARN(rclcpp::get_logger("master_node_logger"), "Joint data timeout");
    }
    else
    {
      auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
      joint_msg->header.stamp = this->now();
      joint_msg->header.frame_id = "arm_link";
      joint_msg->joint_names.push_back("joint_1");
      joint_msg->joint_names.push_back("joint_2");
      joint_msg->joint_names.push_back("joint_3");
      joint_msg->joint_names.push_back("joint_4");
      joint_msg->joint_names.push_back("joint_5");
      joint_msg->joint_names.push_back("joint_6");
      for (uint8_t i = 0; i < 6; i++)
      {
        joint_msg->velocities.push_back((double)jointsVelData.velocities[i] / 100.0);
      }

      joint_pub_->publish(std::move(joint_msg));
    }
  }

public:
  CanMasterNode(const rclcpp::NodeOptions& options) : Node("can_master_node", options)
  {
    jointsVelData.lastReceivedTime = now();
    jointsVelData.velocities.resize(6, 0);

    CAN_driver::init(&master_driver_, "can0");
    CAN_driver::startMasterRead(&master_driver_, &MASTER_HANDLES);

    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());

    write_timer_ = create_wall_timer(std::chrono::microseconds(WRITE_CALLBACK_PERIOD_US),
                                     std::bind(&CanMasterNode::writeCan, this));
    read_timer_ =
        create_wall_timer(std::chrono::microseconds(READ_CALLBACK_PERIOD_US), std::bind(&CanMasterNode::readCan, this));
  }

  ~CanMasterNode() override
  {
  }
};

}  // namespace kalman_master

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(kalman_master::CanMasterNode)