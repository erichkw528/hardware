#ifndef CONTROL_CONVERTER_NODE_HPP_
#define CONTROL_CONVERTER_NODE_HPP_
#include "nav2_util/lifecycle_node.hpp"
#include "control_msgs/msg/pid_state.hpp"
#include "roar_msgs/msg/ego_vehicle_control.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
namespace control_converter
{
    class ControlConverterNode : public nav2_util::LifecycleNode
    {
    public:
        ControlConverterNode();

    protected:
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
        void onLatestAckermannRcvd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

        rclcpp::Publisher<roar_msgs::msg::EgoVehicleControl>::SharedPtr control_publisher_;
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
    };
}

#endif // CONTROL_CONVERTER_NODE_HPP_
