from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "wheel_robot", package_name="wheel_robot_moveit_config"
    ).to_moveit_configs()
    controllers = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    # joint_state_broadcaster first so /joint_states exists before the arms activate.
    ordered = ["joint_state_broadcaster", *controllers]
    return LaunchDescription(
        [
            Node(
                package="wheel_robot_moveit_config",
                executable="spawn_wheel_controllers.py",
                arguments=ordered,
                output="screen",
            )
        ]
    )
