from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
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


def launch_setup(context, *args, **kwargs):
    use_sim = LaunchConfiguration("use_sim")
    # MoveItConfigsBuilder runs xacro immediately (not a launch Substitution), so
    # use_sim must be resolved to a plain string here to pass through as a mapping.
    use_sim_str = use_sim.perform(context)

    pkg_path = FindPackageShare("a6x_ros2_control")

    moveit_config = (
        MoveItConfigsBuilder("a6x_robot", package_name="a6x_moveit_config")
        .robot_description(mappings={"use_sim": use_sim_str})
        .to_moveit_configs()
    )

    robot_controllers = PathJoinSubstitution(
        [pkg_path, "config", "controller", "a6x_control.yaml"]
    )

    #  Get URDF via xacro, selecting real hardware or Isaac Sim via use_sim
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
            " use_sim:=",
            use_sim,
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

    # Delay MoveIt after finger_controller
    delay_moveit_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=finger_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    return [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_after_joint_state_broadcaster,
        delay_finger_controller_after_arm_controller,
        delay_moveit_after_controllers,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Use Isaac Sim (TopicBasedSystem) instead of real hardware",
        )
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
