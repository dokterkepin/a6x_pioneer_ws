from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("diffbot", package_name="a6x_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    position_node = Node(
        package="a6x_commander",
        executable="position",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([position_node])