import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import launch_ros
from pathlib import Path

def generate_launch_description():
    base_path = Path(get_package_share_directory("gokart_hardware_launches"))

    livox_driver = Node(
        package="livox_ros2_driver",
        executable="livox_ros2_driver_node",
        name="livox_lidar",
        output="screen",
        parameters=[
            {
                "xfer_format": 0  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
            },
            {"multi_topic": 1},
            {"data_src": 0},
            {
                "publish_freq": 10.0
            },
            {"output_data_type": 0},
            {
                "frame_id": "center_lidar",
            },
            {
                "user_config_path": (base_path / "param" /"center_livox_lidar_config.json").as_posix()
            },
        ],
    )

    return LaunchDescription(
        [
            livox_driver,
        ]
    )
