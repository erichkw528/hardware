import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_path = Path(get_package_share_directory("gokart_localization"))

    config_path: Path = base_path / "config" / "default_config.yaml"
    robot_localization_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[config_path.as_posix()],
        remappings=[
            ("/gps/fix", "/carla/ego_vehicle/gnss"),
            ("/imu", "/carla/ego_vehicle/imu"),
            ("/odometry/filtered", "/carla/ego_vehicle/odometry"),
            ("/odometry/gps", "output_gps"),
        ],
    )
    ld = launch.LaunchDescription()
    ld.add_action(robot_localization_node)
    return ld
