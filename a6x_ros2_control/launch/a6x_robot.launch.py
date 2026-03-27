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
from moveit_configs_utils import MoveItConfigsBuilder





def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2",
        )
    )
    pkg_path = FindPackageShare("a6x_ros2_control")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("a6x_moveit_config"), "config", "moveit.rviz"]
    )

    moveit_config = (
        MoveItConfigsBuilder("a6x", package_name="a6x_moveit_config").to_moveit_configs()
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
        parameters=[robot_description, robot_controllers],
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
        parameters=[moveit_config.to_dict()],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Delay arm_controller after joint_state_broadcaster
    delay_arm_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    
    # Delay finger_controller after arm_controller
    delay_finger_controller_after_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[finger_controller_spawner],
        )
    )

    # Delay MoveIt and RViz after finger_controller
    delay_moveit_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=finger_controller_spawner,
            on_exit=[move_group_node, rviz_node],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_after_joint_state_broadcaster,
        delay_finger_controller_after_arm_controller,
        delay_moveit_after_controllers,
    ]

    return LaunchDescription(declared_arguments + nodes)
