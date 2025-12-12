from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_6dof", package_name="robot_6dof_moveit_config1").to_moveit_configs()
    return generate_demo_launch(moveit_config)
