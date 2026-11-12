"""Fold both arms into the navigation tuck and close the grippers.

MoveIt plans each arm against the robot body, then executes through the
FollowJointTrajectory controllers. A straight joint trajectory is sent only
when that plan does not succeed. Run this with the base sitting in open space:
the arms swing in from the outstretched zero pose.
"""

import sys

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from wheel_robot_nav.nav_tuck import (
    GRIPPER_CLOSED,
    LEFT_JOINTS,
    RIGHT_JOINTS,
    TUCK_LEFT,
    TUCK_RIGHT,
)


class TuckArms(Node):
    def __init__(self):
        super().__init__("tuck_arms")
        self._joints = {}
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self._move = ActionClient(self, MoveGroup, "move_action")
        self._left = ActionClient(
            self, FollowJointTrajectory, "/left_arm_controller/follow_joint_trajectory"
        )
        self._right = ActionClient(
            self, FollowJointTrajectory, "/right_arm_controller/follow_joint_trajectory"
        )
        self._grip_left = ActionClient(
            self, GripperCommand, "/left_gripper_controller/gripper_cmd"
        )
        self._grip_right = ActionClient(
            self, GripperCommand, "/right_gripper_controller/gripper_cmd"
        )

    def _on_joints(self, msg):
        self._joints = dict(zip(msg.name, msg.position))

    def run(self):
        if not self._move.wait_for_server(timeout_sec=90.0):
            self.get_logger().error("move_group is not available")
            return False
        self._wait_for_joints()
        self._close_grippers()
        left = self._tuck("left_arm", LEFT_JOINTS, TUCK_LEFT, self._left)
        right = self._tuck("right_arm", RIGHT_JOINTS, TUCK_RIGHT, self._right)
        if left and right:
            self.get_logger().info("both arms are tucked; the base can navigate")
        return left and right

    def _wait_for_joints(self):
        for _ in range(100):
            if any(name in self._joints for name in LEFT_JOINTS):
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn("no /joint_states yet; planning from MoveIt's current state")

    def _close_grippers(self):
        goal = GripperCommand.Goal()
        goal.command.position = GRIPPER_CLOSED
        goal.command.max_effort = 40.0
        for client, name in (
            (self._grip_left, "left gripper"),
            (self._grip_right, "right gripper"),
        ):
            if not client.wait_for_server(timeout_sec=15.0):
                self.get_logger().warn(f"{name} controller is not available")
                continue
            self._wait_action(client, goal, 20.0, name)

    def _tuck(self, group, names, positions, trajectory_client):
        self.get_logger().info(f"planning {group} to the navigation tuck")
        if self._plan_and_execute(group, names, positions):
            self.get_logger().info(f"{group} tucked")
            return True
        self.get_logger().warn(f"MoveIt did not tuck {group}; sending a joint trajectory")
        return self._trajectory(trajectory_client, names, positions, group)

    def _plan_and_execute(self, group, names, positions):
        request = MotionPlanRequest()
        request.group_name = group
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.15
        request.max_acceleration_scaling_factor = 0.15
        request.pipeline_id = "ompl"
        request.start_state.is_diff = True
        request.workspace_parameters.header.frame_id = "odom"
        for axis in ("x", "y", "z"):
            setattr(request.workspace_parameters.min_corner, axis, -2.0)
            setattr(request.workspace_parameters.max_corner, axis, 2.0)
        constraints = Constraints()
        for name, position in zip(names, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = 0.02
            joint.tolerance_below = 0.02
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        request.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        result = self._wait_action(self._move, goal, 90.0, group)
        if result is None:
            return False
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f"{group} MoveIt error code {code}")
            return False
        return True

    def _trajectory(self, client, names, positions, label):
        if not client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error(f"{label} trajectory controller is not available")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        start = JointTrajectoryPoint()
        start.positions = [float(self._joints.get(name, 0.0)) for name in names]
        start.time_from_start.sec = 0
        end = JointTrajectoryPoint()
        end.positions = [float(value) for value in positions]
        end.time_from_start.sec = 8
        goal.trajectory.points = [start, end]
        result = self._wait_action(client, goal, 30.0, label)
        if result is None:
            return False
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f"{label} trajectory failed with code {result.error_code}")
            return False
        self.get_logger().info(f"{label} tucked")
        return True

    def _wait_action(self, client, goal, timeout, label):
        accepted = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, accepted, timeout_sec=timeout)
        handle = accepted.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label} goal was rejected")
            return None
        finished = handle.get_result_async()
        rclpy.spin_until_future_complete(self, finished, timeout_sec=timeout)
        wrapped = finished.result()
        if wrapped is None:
            self.get_logger().error(f"{label} timed out")
            return None
        return wrapped.result


def main():
    rclpy.init()
    node = TuckArms()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
