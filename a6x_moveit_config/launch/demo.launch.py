from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("a6x_robot", package_name="a6x_moveit_config")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner", "ompl"], default_planning_pipeline="pilz_industrial_motion_planner")
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
