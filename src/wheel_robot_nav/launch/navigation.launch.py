"""16-line lidar localization, swerve drive, and Nav2.

``lidar_localization`` predicts with wheel speed and the IMU gyro, then
corrects the pose with point-to-plane matching on ``/points``. It publishes
``map`` -> ``odom`` -> ``base_link`` and the occupancy grid. Nav2 plans with
Smac 2D and tracks with omnidirectional MPPI. The costmaps read the same cloud.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("wheel_robot_nav")
    nav2 = get_package_share_directory("nav2_bringup")
    urdf = os.path.join(share, "urdf", "wheel_robot_nav.urdf.xacro")
    params = os.path.join(share, "config", "nav2_params.yaml")
    description = ParameterValue(Command(["xacro ", urdf]), value_type=str)

    robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": description, "use_sim_time": True}],
        remappings=[("joint_states", "/isaac_joint_states")],
    )
    odometry = Node(
        package="wheel_robot_nav",
        executable="wheel_odometry",
        parameters=[{"use_sim_time": True, "publish_tf": False}],
    )
    localization = Node(
        package="wheel_robot_nav",
        executable="lidar_localization",
        parameters=[{"use_sim_time": True}],
    )
    swerve = Node(
        package="wheel_robot_nav",
        executable="swerve_controller",
        parameters=[{"use_sim_time": True}],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": "True",
            "autostart": "True",
            "params_file": params,
            "use_composition": "False",
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(nav2, "rviz", "nav2_default_view.rviz")],
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription([robot_state, odometry, localization, swerve, navigation, rviz])
