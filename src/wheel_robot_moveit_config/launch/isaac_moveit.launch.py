"""Launch MoveIt + ros2_control TopicBasedSystem for Isaac Sim.

Prerequisites:
1. Build USD:  bash scripts/build_wheel_robot_isaac_usd.sh
2. Open wheel_robot_moveit.usd in Isaac Sim and press Play
   (publishes /isaac_joint_states, subscribes /isaac_joint_commands)
3. Then run this launch.
"""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "wheel_robot", package_name="wheel_robot_moveit_config"
    ).to_moveit_configs()
    return generate_demo_launch(moveit_config)
