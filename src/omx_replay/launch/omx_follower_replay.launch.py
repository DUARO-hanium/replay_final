#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port_name = LaunchConfiguration("port_name")
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("open_manipulator_description"),
            "urdf", "omx_f", "omx_f.urdf.xacro",
        ]),
        " port_name:=", port_name,
    ])
    controller_config = PathJoinSubstitution([
        FindPackageShare("omx_replay"), "config", "replay_controller.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "port_name", default_value="/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_6AFAC93B5157375037202020FF0F272E-if00",
            description="Follower hardware port.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, controller_config],
            output="both",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["replay_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ])
