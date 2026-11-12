"""Navigation, arm tuck, and MoveIt in one RViz window.

The robot model has no fixed world link. ``lidar_localization`` publishes
``map`` -> ``odom`` -> ``base_link``, and a planar virtual joint makes ``odom``
MoveIt's planning frame. Do not launch ``isaac_moveit.launch.py`` or
``navigation.launch.py`` at the same time: each starts its own robot state
publisher or a fixed ``world`` frame.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _mobile_moveit_config():
    config = (
        MoveItConfigsBuilder("wheel_robot", package_name="wheel_robot_moveit_config")
        .robot_description(file_path="config/wheel_robot_mobile.urdf.xacro")
        .to_moveit_configs()
    )
    semantic = config.robot_description_semantic["robot_description_semantic"]
    if not isinstance(semantic, str):
        raise RuntimeError("MoveIt SRDF was not loaded as text")
    opening = '<robot name="wheel_robot">'
    virtual = (
        opening
        + '\n  <virtual_joint name="virtual_joint" type="planar" '
        + 'parent_frame="odom" child_link="base_link"/>'
    )
    if "virtual_joint" not in semantic:
        semantic = semantic.replace(opening, virtual, 1)
    config.robot_description_semantic["robot_description_semantic"] = semantic
    return config


def generate_launch_description():
    nav_share = get_package_share_directory("wheel_robot_nav")
    moveit_share = get_package_share_directory("wheel_robot_moveit_config")
    nav2_share = get_package_share_directory("nav2_bringup")
    moveit = _mobile_moveit_config()
    use_sim = {"use_sim_time": True}

    robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit.robot_description, use_sim],
        remappings=[("joint_states", "/isaac_joint_states")],
    )
    odometry = Node(
        package="wheel_robot_nav",
        executable="wheel_odometry",
        parameters=[{"publish_tf": False, **use_sim}],
    )
    localization = Node(
        package="wheel_robot_nav",
        executable="lidar_localization",
        parameters=[use_sim],
    )
    swerve = Node(
        package="wheel_robot_nav",
        executable="swerve_controller",
        parameters=[use_sim],
    )
    control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit.robot_description,
            os.path.join(moveit_share, "config", "ros2_controllers.yaml"),
            use_sim,
        ],
        output="screen",
    )
    spawn = Node(
        package="wheel_robot_moveit_config",
        executable="spawn_wheel_controllers.py",
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
                **use_sim,
            },
        ],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "autostart": "True",
            "params_file": os.path.join(nav_share, "config", "nav2_params.yaml"),
            "use_composition": "False",
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(nav_share, "config", "nav_moveit.rviz")],
        parameters=[
            moveit.robot_description,
            moveit.robot_description_semantic,
            moveit.robot_description_kinematics,
            moveit.planning_pipelines,
            moveit.joint_limits,
            use_sim,
        ],
    )
    tuck = Node(
        package="wheel_robot_nav",
        executable="tuck_arms",
        parameters=[use_sim],
        output="screen",
        condition=IfCondition(LaunchConfiguration("tuck_arms")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "tuck_arms",
                default_value="true",
                description="Fold both arms beside the chassis before driving",
            ),
            robot_state,
            odometry,
            localization,
            swerve,
            control,
            spawn,
            move_group,
            navigation,
            rviz,
            tuck,
        ]
    )
