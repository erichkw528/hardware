
#include "pid_control_converter/pid_control_converter_node.hpp"
using std::placeholders::_1;

namespace pid_control_converter
{
  PIDControlConverterNode::PIDControlConverterNode() : Node("pid_control_converter_node")
  {
    steering_pid_sub_ = this->create_subscription<control_msgs::msg::PidState>(
        "steering_pid_topic", 10, std::bind(&PIDControlConverterNode::on_steering_pid_state_recv, this, _1));
    throttle_pid_sub_ = this->create_subscription<control_msgs::msg::PidState>(
        "throttle_pid_topic", 10, std::bind(&PIDControlConverterNode::on_throttle_pid_state_recv, this, _1));
    control_publisher_ = this->create_publisher<roar_msgs::msg::EgoVehicleControl>("output_topic", 10);
    this->declare_parameter("loop_rate_millis", 10);
    this->publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(this->get_parameter("loop_rate_millis").as_int()), 
      std::bind(&PIDControlConverterNode::publish_timer_callback, this));
  }

  void PIDControlConverterNode::publish_timer_callback()
  {
    if (this->latest_steering_msg && this->latest_throttle_msg)
    {

      roar_msgs::msg::EgoVehicleControl msg = roar_msgs::msg::EgoVehicleControl();
      msg.throttle = this->latest_throttle_msg->output;
      msg.steer = this->latest_steering_msg->output;
      this->control_publisher_->publish(msg);
      return;
    }

  }

  void PIDControlConverterNode::on_steering_pid_state_recv(const control_msgs::msg::PidState::SharedPtr msg)
  {
    this->latest_steering_msg = msg;
  }

  void PIDControlConverterNode::on_throttle_pid_state_recv(const control_msgs::msg::PidState::SharedPtr msg)
  {
    this->latest_throttle_msg = msg;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pid_control_converter::PIDControlConverterNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}