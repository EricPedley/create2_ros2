#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "device_path",
            default_value="/dev/ttyUSB0",
            description="Serial device path for Create 2",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Baud rate for Create 2 communication",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations.",
        )
    )

    # Initialize Arguments
    device_path = LaunchConfiguration("device_path")
    baud_rate = LaunchConfiguration("baud_rate")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("create2_ros2"), "urdf", "create2.urdf.xacro"]
            ),
            " ",
            "device_path:=",
            device_path,
            " ",
            "baud_rate:=",
            baud_rate,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot controllers config file
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("create2_ros2"),
            "config",
            "create2_ros2_control.yaml",
        ]
    )

    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Diff drive controller spawner
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["create2_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        )
    )

    nodes = [
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
    ]

    #print(f'Use fake hardware: {use_fake_hardware}', type(use_fake_hardware))
    #if use_fake_hardware == 'false':
    nodes.append(control_node)
    nodes.append(delay_robot_controller_spawner_after_joint_state_broadcaster_spawner)

    return LaunchDescription(declared_arguments + nodes)
