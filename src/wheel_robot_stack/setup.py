from setuptools import setup

package_name = "wheel_robot_stack"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="Pallet pattern and suction pick-and-place for the wheel robot in Isaac Sim.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "stack_boxes = wheel_robot_stack.stack_boxes:main",
        ],
    },
)
