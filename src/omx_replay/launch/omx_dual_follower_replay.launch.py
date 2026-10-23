#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def follower_group(namespace, port, system_name):
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("omx_replay"), "ros2_control", "omx_f_replay.urdf.xacro",
        ]),
        " port_name:=", port,
        " system_name:=", system_name,
    ])
    controller_config = PathJoinSubstitution([
        FindPackageShare("omx_replay"), "config", "replay_controller.yaml",
    ])
    manager = f"/{namespace}/controller_manager"

    return GroupAction([
        PushRosNamespace(namespace),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
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
            arguments=["joint_state_broadcaster", "--controller-manager", manager],
            output="screen",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["replay_controller", "--controller-manager", manager],
            output="screen",
        ),
    ])


def generate_launch_description():
    port1 = LaunchConfiguration("port1")
    port2 = LaunchConfiguration("port2")
    return LaunchDescription([
        DeclareLaunchArgument("port1", description="First follower port or by-id path."),
        DeclareLaunchArgument("port2", description="Second follower port or by-id path."),
        follower_group("follower1", port1, "OMXFReplaySystem1"),
        follower_group("follower2", port2, "OMXFReplaySystem2"),
    ])
