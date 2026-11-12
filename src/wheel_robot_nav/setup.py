from setuptools import setup

package_name = "wheel_robot_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/navigation.launch.py",
                "launch/navigation_moveit.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/nav2_params.yaml",
                "config/ekf.yaml",
                "config/nav_moveit.rviz",
            ],
        ),
        ("share/" + package_name + "/urdf", ["urdf/wheel_robot_nav.urdf.xacro"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="Swerve-drive lidar and IMU localization and Nav2 bringup for the wheel robot in Isaac Sim.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "swerve_controller = wheel_robot_nav.swerve_controller:main",
            "wheel_odometry = wheel_robot_nav.wheel_odometry:main",
            "odom_fusion = wheel_robot_nav.odom_fusion:main",
            "imu_covariance = wheel_robot_nav.imu_covariance:main",
            "lidar_localization = wheel_robot_nav.lidar_localization:main",
            "tuck_arms = wheel_robot_nav.tuck_arms:main",
        ],
    },
)
