import os
import json
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    dir = FindPackageShare("robotiq_2f_bringup").find("robotiq_2f_bringup")
    robotiq_description_dir = FindPackageShare("robotiq_2f_description").find("robotiq_2f_description")

    scenario_path = {
        "scenario_path": os.path.join(dir, "scenario"), 
    }

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robotiq_description_package",
            default_value="robotiq_2f_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robotiq_description_file",
            default_value="robotiq_2f_85_model.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    robotiq_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(robotiq_description_dir, "urdf", "robotiq_2f_85_model.xacro")
        ]
    )

    robotiq_ghost_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(robotiq_description_dir, "urdf", "ghost_robotiq_2f_85_model.xacro"),
            " ",
            "prefix:=ghost_"
        ]
    )

    robotiq_robot_description = {"robot_description": robotiq_robot_description_content}
    robotiq_ghost_robot_description = {"robot_description": robotiq_ghost_robot_description_content}

    rviz_config_file = os.path.join(dir, "config", "bringup.rviz")

    robotiq_robot_parameters = {
        "urdf_raw": robotiq_robot_description_content,
        "initial_joint_state": ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0"], # fully open
        "initial_base_link_id": "robotiq_2f_base_link",
        "initial_face_plate_id": "left_inner_finger_pad",
        "initial_tcp_id": "left_inner_finger_pad"
    }

    robotiq_ghost_robot_parameters = {
        "urdf_raw": robotiq_ghost_robot_description_content,
        "initial_joint_state": ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0"], # fully open
        "initial_base_link_id": "ghost_" + "robotiq_2f_base_link",
        "initial_face_plate_id": "ghost_" + "left_inner_finger_pad",
        "initial_tcp_id": "ghost_" + "left_inner_finger_pad"
    }

    robotiq_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robotiq",
        output="screen",
        parameters=[robotiq_robot_description],
        # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    robotiq_ghost_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robotiq/ghost",
        output="screen",
        parameters=[robotiq_ghost_robot_description],
        emulate_tty=True,
    )

    robotiq_teaching_ghost_node = Node(
        package="teaching_ghost",
        executable="teaching_ghost",
        namespace="robotiq",
        output="screen",
        parameters=[robotiq_ghost_robot_parameters],
        emulate_tty=True,
    )

    teaching_marker_node = Node(
        package="teaching_marker",
        executable="teaching_marker",
        namespace="",
        output="screen",
        parameters=[robotiq_ghost_robot_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    robotiq_simple_robot_simulator_node = Node(
        package="simple_robot_simulator",
        executable="simple_robot_simulator",
        namespace="robotiq",
        output="screen",
        parameters=[robotiq_robot_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    tf_lookup_node = Node(
        package="tf_lookup",
        executable="tf_lookup",
        namespace="",
        output="screen",
        parameters=[scenario_path],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    tfbc_node = Node(
        package="simple_robot_simulator_tfbc",
        executable="broadcaster",
        namespace="",
        output="screen",
        parameters=[scenario_path],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    nodes_to_start = [
        robotiq_robot_state_publisher_node,
        robotiq_simple_robot_simulator_node,
        robotiq_ghost_robot_state_publisher_node,
        robotiq_teaching_ghost_node,
        rviz_node,
        tfbc_node,
        # teaching_marker_node,
        tf_lookup_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
