#ifndef PID_CONTROL_CONVERTER_NODE_HPP_
#define PID_CONTROL_CONVERTER_NODE_HPP_
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "control_msgs/msg/pid_state.hpp"
#include "roar_msgs/msg/ego_vehicle_control.hpp"
namespace pid_control_converter
{
    class PIDControlConverterNode : public rclcpp::Node
    {
        public:
            PIDControlConverterNode();        
        protected:
            void on_steering_pid_state_recv(const control_msgs::msg::PidState::SharedPtr msg);
            void on_throttle_pid_state_recv(const control_msgs::msg::PidState::SharedPtr msg);
            void publish_timer_callback();

            rclcpp::Publisher<roar_msgs::msg::EgoVehicleControl>::SharedPtr control_publisher_;
            rclcpp::TimerBase::SharedPtr publish_timer_;

            rclcpp::Subscription<control_msgs::msg::PidState>::SharedPtr steering_pid_sub_;
            rclcpp::Subscription<control_msgs::msg::PidState>::SharedPtr throttle_pid_sub_;

            std::shared_ptr<control_msgs::msg::PidState> latest_steering_msg;
            std::shared_ptr<control_msgs::msg::PidState> latest_throttle_msg;
    };
}

#endif // PID_CONTROL_CONVERTER_NODE_HPP_
