import os
from pathlib import Path

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration


def _make_rsp_node(urdf_filename: str) -> launch_ros.actions.Node:
    """
    Create a robot_state_publisher node and populate the robot_description with the contents of the specified URDF file.
    """
    urdf_path = Path(
        get_package_share_directory("avalue_robot_urdf")
    ) / "urdf" / urdf_filename

    return launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": urdf_path.read_text(encoding="utf-8")}],
    )


def generate_launch_description():

    # AMR Type - mini_mec
    mini_mec = GroupAction(
        [
            _make_rsp_node("mini_mec_robot.urdf"),
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser",
                arguments=[
                    "0.0",
                    "0",
                    "0.18",
                    "0",
                    "0",
                    "0",
                    "base_footprint",
                    "laser",
                ],
            ),
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera",
                arguments=[
                    "0.195",
                    "0",
                    "0.35",
                    "0",
                    "0",
                    "0",
                    "base_footprint",
                    "camera_link",
                ],
            ),
        ]
    )

    # AMR Type - mini_tank
    mini_tank = GroupAction(
        [
            _make_rsp_node("mini_diff_robot.urdf"),
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser",
                arguments=[
                    "0.00",
                    "0",
                    "0.155",
                    "3.1415",
                    "0",
                    "0",
                    "base_footprint",
                    "laser",
                ],
            ),
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera",
                arguments=[
                    "0.14",
                    "0",
                    "0.25",
                    "0",
                    "0",
                    "0",
                    "base_footprint",
                    "camera_link",
                ],
            ),
        ]
    )

    # AMR Type - mini_4wd
    mini_4wd = GroupAction(
        [
            _make_rsp_node("mini_diff_robot.urdf"),
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser",
                arguments=[
                    "0.0",
                    "0",
                    "0.155",
                    "0",
                    "0",
                    "0",
                    "base_footprint",
                    "laser",
                ],
            ),
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera",
                arguments=[
                    "0.195",
                    "0",
                    "0.25",
                    "0",
                    "0",
                    "0",
                    "base_footprint",
                    "camera_link",
                ],
            ),
        ]
    )

    # AMR Type - mini_diff
    mini_diff = GroupAction(
        [
            _make_rsp_node("mini_diff_robot.urdf"),
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser",
                arguments=[
                    "0.0",
                    "0",
                    "0.155",
                    "3.1415",
                    "0",
                    "0",
                    "base_footprint",
                    "laser",
                ],
            ),
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera",
                arguments=[
                    "0.0",
                    "0",
                    "0.25",
                    "0",
                    "0",
                    "0",
                    "base_footprint",
                    "camera_link",
                ],
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Current AMR Type mini_mec
    # options: mini_diff, mini_4wd, mini_tank, mini_mec
    ld.add_action(mini_mec)

    return ld
