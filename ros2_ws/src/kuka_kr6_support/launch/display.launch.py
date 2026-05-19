import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("kuka_kr6_support"), "urdf", "kr6r900sixx.xacro"]
    )

    robot_description = {
        "robot_description": Command(["xacro ", xacro_file])
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )
    initial_joint_positions = {
        "joint_a1": 0.080,
        "joint_a2": -1.609,
        "joint_a3": 1.603,
        "joint_a4": 0.0,
        "joint_a5": 0.0,
        "joint_a6": 0.0,
    }
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{
            "zeros": initial_joint_positions
        }]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])

