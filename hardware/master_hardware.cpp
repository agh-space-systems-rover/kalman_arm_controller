#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "kalman_arm_controller/can_lib/can_driver.hpp"
#include <future>
#include <chrono>

const uint16_t WRITE_CALLBACK_PERIOD_US = 10;
const uint16_t READ_CALLBACK_PERIOD_US = 10;

namespace kalman_master
{

class CanMasterNode : public rclcpp::Node
{
private:
  MasterCanDriver can_driver_ = MasterCanDriver("can0");
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
        std::lock_guard<std::mutex> lock(this->can_driver_.m_write);
        // Do something related to variables that write uses
      }
      // Run write in a separate thread
      writer = std::async(std::launch::async, [&] { this->can_driver_.write(); });
    }
  }

  void readCan()
  {
    std::lock_guard<std::mutex> lock(this->can_driver_.m_read);
    // Do something related to variables that read uses
  }

public:
  CanMasterNode(const rclcpp::NodeOptions& options) : Node("can_master_node", options)
  {
    can_driver_.init();

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
