#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    port_name = LaunchConfiguration("port_name")

    robot_description = Command([
        PathJoinSubstitution([
            FindExecutable(name="xacro")
        ]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("omx_replay"),
            "ros2_control",
            "omx_l_replay.urdf.xacro",
        ]),
        " ",
        "port_name:=",
        port_name,
    ])

    controller_config = PathJoinSubstitution([
        FindPackageShare("omx_replay"),
        "config",
        "replay_controller.yaml",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controller_config,
        ],
        remappings=[
            ("robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    replay_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "replay_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "port_name",
            default_value="/dev/ttyACM0",
        ),

        robot_state_publisher,
        control_node,
        joint_state_spawner,
        replay_controller_spawner,
    ])
