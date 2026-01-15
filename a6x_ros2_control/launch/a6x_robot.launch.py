from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    pkg_path = FindPackageShare("a6x_ros2_control")
    gui = LaunchConfiguration("gui")

    rviz_config_file = PathJoinSubstitution(
        [pkg_path, "config", "rviz", "a6x_config.rviz"]
    )
    robot_controllers = PathJoinSubstitution(
        [pkg_path, "config", "controller", "a6x_control.yaml"]
    )

    #  Get URDF via xacro, passing the eth_device argument
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    pkg_path,
                    "description",
                    "urdf",
                    "a6x.urdf.xacro",
                ]
            ),
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--param-file",
            robot_controllers,
        ],
    )
    
    finger_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "finger_controller",
            "--param-file",
            robot_controllers,
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Delay start of joint_state_broadcaster after arm_controller
    delay_joint_state_broadcaster_after_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    # Delay finger_controller after joint_state_broadcaster
    delay_finger_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[finger_controller_spawner],
        )
    )

    # Delay rviz start after finger_controller
    delay_rviz_after_finger_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=finger_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        arm_controller_spawner,
        delay_joint_state_broadcaster_after_arm_controller,
        delay_finger_controller_after_joint_state_broadcaster,
        delay_rviz_after_finger_controller,
    ]

    return LaunchDescription(declared_arguments + nodes)
