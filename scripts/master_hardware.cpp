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
#include <thread>
extern "C" {
#include "kalman_arm_controller/can_libs/can_driver.hpp"
}
#include <future>
#include <chrono>

const uint16_t WRITE_CALLBACK_PERIOD_US = 10;
const uint16_t READ_CALLBACK_PERIOD_US = 10;

namespace kalman_master
{

class CanMasterNode : public rclcpp::Node
{
private:
  CAN_driver::DriverVars_t master_driver_;
  std::future<void> writer;
  rclcpp::TimerBase::SharedPtr write_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;

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
  }

public:
  CanMasterNode(const rclcpp::NodeOptions& options) : Node("can_master_node", options)
  {
    CAN_driver::init(&master_driver_, "can0");
    CAN_driver::startMasterRead(&master_driver_);

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