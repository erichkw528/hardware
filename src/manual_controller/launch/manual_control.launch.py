from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from pathlib import Path


def generate_launch_description():
    loop_rate = DeclareLaunchArgument("loop_rate", default_value="0.05")

    manual_control_node = Node(
        package="manual_controller",
        executable="manual_controller_node",
        name="manual_control",
        parameters=[{"loop_rate": LaunchConfiguration("loop_rate")}],
    )
    ld = LaunchDescription()

    ld.add_action(loop_rate)
    ld.add_action(manual_control_node)
    return ld
