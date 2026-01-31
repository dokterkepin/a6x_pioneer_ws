from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the a6x MoveIt Configuration
    # This matches the builder used in your a6x_demo.py
    moveit_config = (
        MoveItConfigsBuilder("a6x")
        .robot_description(file_path="config/a6x_robot.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 2. Run the MTC Node with the config
    mtc_node = Node(
        package="mtc_tutorial",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([
        mtc_node
    ])