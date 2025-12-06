from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    ]

    pkg_path = FindPackageShare("a6x_description")
    gui = LaunchConfiguration("gui")

    rviz_config_file = PathJoinSubstitution(
        [pkg_path, "config", "rviz", "a6x_config.rviz"]
    )

    urdf_file = PathJoinSubstitution([pkg_path, "urdf", "a6x.urdf"])

    robot_description_content = Command(["cat ", urdf_file])
    robot_description = {"robot_description": robot_description_content}

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="log",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    return LaunchDescription(declared_arguments + [rsp_node, gui_node, rviz_node])

    
