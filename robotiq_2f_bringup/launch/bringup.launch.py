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
    description_dir = FindPackageShare("robotiq_2f_description").find("robotiq_2f_description")

    scenario_path = {
        "scenario_path": os.path.join(dir, "scenario"), 
    }

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="robotiq_2f_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="robotiq_2f_85_model.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(description_dir, "urdf", "robotiq_2f_85_model.xacro")
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = os.path.join(dir, "config", "bringup.rviz")

    robot_parameters = {
        "urdf_raw": robot_description_content,
        # "initial_joint_state": ["0.0", "-1.5707", "1.5707", "-1.5707", "-1.5707", "0.0"],
        "initial_base_link_id": "robotiq_2f_base_link",
        "initial_face_plate_id": "left_inner_finger_pad",
        "initial_tcp_id": "left_inner_finger_pad"
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="",
        output="screen",
        parameters=[robot_description],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    simple_robot_simulator_node = Node(
        package="simple_robot_simulator",
        executable="simple_robot_simulator",
        namespace="",
        output="screen",
        parameters=[robot_parameters],
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
        robot_state_publisher_node,
        simple_robot_simulator_node,
        rviz_node,
        tfbc_node
    ]

    return LaunchDescription(nodes_to_start)
