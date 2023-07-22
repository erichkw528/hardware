#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,Command
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    camera_model = "zed2i"
    camera_name = "zed2i"

    # Set LOG format
    os.environ[
        "RCUTILS_CONSOLE_OUTPUT_FORMAT"
    ] = "{time} [{name}] [{severity}] {message}"

    # Define LaunchDescription variable
    ld = LaunchDescription()
    base_frame = LaunchConfiguration("base_frame")
    declare_base_frame_cmd = DeclareLaunchArgument(
        "base_frame", default_value="base_link", description="Name of the base link."
    )
    ld.add_action(declare_base_frame_cmd)

    # Configuration variables
    publish_urdf = "true"  # Publish static frames from camera URDF
    # Position X of the camera with respect to the base frame [m].
    cam_pos_x = "0.0"
    # Position Y of the camera with respect to the base frame [m].
    cam_pos_y = "0.0"
    # Position Z of the camera with respect to the base frame [m].
    cam_pos_z = "0.0"
    # Roll orientation of the camera with respect to the base frame [rad].
    cam_roll = "0.0"
    # Pitch orientation of the camera with respect to the base frame [rad].
    cam_pitch = "0.0"
    # Yaw orientation of the camera with respect to the base frame [rad].
    cam_yaw = "0.0"

    # ZED Configurations to be loaded by ZED Node
    config_common_path = os.path.join(
        get_package_share_directory("gokart_hardware_launches"), "param", "zed_common.yaml"
    )
    config_camera_path = os.path.join(
       get_package_share_directory("gokart_hardware_launches"), "param", "zed2i_config.yaml"
    )

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(
        get_package_share_directory("zed_wrapper"), "urdf", "zed_descr.urdf.xacro"
    )

    # ZED Wrapper node
    zed_wrapper_node = Node(
        package="zed_wrapper",
        namespace=camera_name,
        executable="zed_wrapper",
        name="center_zed",
        output="screen",

        remappings=[
                ('/zed2i/center_zed/odom', '/roar/odometry'),
                ('/zed2i/center_zed/depth/camera_info', '/roar/front/depth/camera_info'),
                ('/zed2i/center_zed/left/camera_info', '/roar/front/rgb/camera_info'),
                ('/zed2i/center_zed/depth/depth_registered', '/roar/front/depth/image'),
                ('/zed2i/center_zed/left/image_rect_color', '/roar/front/rgb/image'),
                ('/zed2i/center_zed/imu/data', '/roar/imu'),
        ],
        parameters=[
            # YAML files
            config_common_path,  # Common parameters
            config_camera_path,  # Camera related parameters
            # Overriding
            {
                "general.camera_name": camera_name,
                "general.camera_model": camera_model,
                "general.svo_file": "live",
                "pos_tracking.base_frame": base_frame,
                "general.zed_id": 0,
            },
        ],
    )


    # Robot State Publisher node
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package="robot_state_publisher",
        namespace=camera_name,
        executable="robot_state_publisher",
        name="zed_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro",
                        " ",
                        xacro_path,
                        " ",
                        "camera_name:=",
                        camera_name,
                        " ",
                        "camera_model:=",
                        camera_model,
                        " ",
                        "base_frame:=",
                        base_frame,
                        " ",
                        "cam_pos_x:=",
                        cam_pos_x,
                        " ",
                        "cam_pos_y:=",
                        cam_pos_y,
                        " ",
                        "cam_pos_z:=",
                        cam_pos_z,
                        " ",
                        "cam_roll:=",
                        cam_roll,
                        " ",
                        "cam_pitch:=",
                        cam_pitch,
                        " ",
                        "cam_yaw:=",
                        cam_yaw,
                    ]
                )
            }
        ],
    )

    ld.add_action(zed_wrapper_node)
    ld.add_action(rsp_node)



    return ld
