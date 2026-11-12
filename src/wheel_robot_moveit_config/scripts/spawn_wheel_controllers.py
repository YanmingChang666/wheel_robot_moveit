#!/usr/bin/env python3
"""Load, configure, and activate controllers one at a time.

Parallel spawners race the controller manager: a timed-out load is retried,
the second call returns "already loaded", and that spawner exits before
configure/activate. This script treats an already-loaded controller as success
and then configures and activates it.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from controller_manager import (
    configure_controller,
    list_controllers,
    load_controller,
    switch_controllers,
)
from controller_manager.controller_manager_services import ServiceNotFoundError


CALL_TIMEOUT = 30.0
SWITCH_TIMEOUT = 30.0


def _list(node, manager):
    return list_controllers(node, manager, 0.0, CALL_TIMEOUT).controller


def _find(node, manager, name):
    for controller in _list(node, manager):
        if controller.name == name:
            return controller
    return None


def _ensure_active(node, manager, name):
    controller = _find(node, manager, name)
    if controller is None:
        result = load_controller(node, manager, name, 0.0, CALL_TIMEOUT)
        controller = _find(node, manager, name)
        if controller is None:
            node.get_logger().fatal(
                f"Failed loading controller {name} (ok={getattr(result, 'ok', None)})"
            )
            return False
        node.get_logger().info(f"Loaded {name}")
    else:
        node.get_logger().info(f"{name} already loaded ({controller.state}), continuing")

    if controller.state == "unconfigured":
        result = configure_controller(node, manager, name, 0.0, CALL_TIMEOUT)
        if not result.ok:
            controller = _find(node, manager, name)
            if controller is None or controller.state == "unconfigured":
                node.get_logger().fatal(f"Failed to configure {name}")
                return False
        else:
            controller = _find(node, manager, name)

    if controller is not None and controller.state == "active":
        node.get_logger().info(f"{name} is already active")
        return True

    result = switch_controllers(
        node,
        manager,
        [],
        [name],
        True,
        True,
        SWITCH_TIMEOUT,
        CALL_TIMEOUT,
    )
    if not result.ok:
        controller = _find(node, manager, name)
        if controller is not None and controller.state == "active":
            node.get_logger().info(f"{name} became active")
            return True
        node.get_logger().fatal(f"Failed to activate {name}")
        return False

    node.get_logger().info(f"Configured and activated {name}")
    return True


def main():
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if not argv:
        argv = [
            "joint_state_broadcaster",
            "left_arm_controller",
            "right_arm_controller",
            "torso_controller",
            "left_gripper_controller",
            "right_gripper_controller",
        ]

    node = Node("spawn_wheel_controllers")
    manager = "/controller_manager"
    failed = []
    try:
        for name in argv:
            try:
                if not _ensure_active(node, manager, name):
                    failed.append(name)
            except (ServiceNotFoundError, RuntimeError) as exc:
                node.get_logger().fatal(f"{name}: {exc}")
                failed.append(name)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
