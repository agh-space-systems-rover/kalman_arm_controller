#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
// #include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>
#include "kalman_interfaces/msg/master_message.hpp"
#include <format>

// We'll just set up parameters here
const std::string JOY_TOPIC =
    "/master_com/master_to_ros/x" + std::format("{:x}", kalman_interfaces::msg::MasterMessage().ARM_SET_JOINTS);
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";

namespace arm_master
{
class MasterToServo : public rclcpp::Node
{
public:
  MasterToServo(const rclcpp::NodeOptions& options) : Node("servo_publisher", options)
  {
    // Setup pub/sub
    joy_sub_ = this->create_subscription<kalman_interfaces::msg::MasterMessage>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const kalman_interfaces::msg::MasterMessage::ConstSharedPtr& msg) { return joyCB(msg); });

    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  ~MasterToServo() override
  {
  }

  void joyCB(kalman_interfaces::msg::MasterMessage::ConstSharedPtr msg)
  {
    // Create the messages we might publish
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    for (uint8_t i = 0; i < 6; i++)
    {
      joint_msg->joint_names.push_back("joint_" + std::to_string(i + 1));
      joint_msg->velocities.push_back(double(msg->data[i] - 128) / 128.0);
    }
  }

private:
  rclcpp::Subscription<kalman_interfaces::msg::MasterMessage>::SharedPtr joy_sub_;

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
};  // class MasterToServo

}  // namespace arm_master

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(arm_master::MasterToServo)
