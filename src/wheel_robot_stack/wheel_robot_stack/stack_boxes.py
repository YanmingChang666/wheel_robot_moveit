"""Pick from one fixed station and place onto the pallet pattern.

Isaac Sim plays ``wheel_robot_suction_stack.usd``. This node plans with
MoveIt, pulses the left suction cup, and asks the stage to spawn the next
box only after the current one has been released on its slot.
"""

import math
import random
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from wheel_robot_nav.kinematics import MODULES, steer_error, twist_to_wheels
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Int32

from wheel_robot_stack.pattern import (
    APPROACH_HEIGHT,
    BOX,
    LIFT_LIMIT,
    STAND_SIZE,
    Slot,
    base_for_slot,
    carried_offset,
    lift_for_tool_z,
    pickup_box_top,
    pallet_center,
    pallet_size,
    pallet_slots,
    pickup_center,
    stand_center,
    tool_z_for_pick,
    tool_z_for_place,
)

# MoveIt still carries the Robotiq meshes. In the suction stage those prims are
# inactive, so the wrist and gripper may overlap the box at the seal pose.
EE_LINKS = ["Link5_L", "Link6_L", "Link7_L", "tool0_L"]
GRIPPER_LINKS = [
    "left_robotiq_coupler",
    "left_robotiq_85_base_link",
    "left_robotiq_85_left_finger_link",
    "left_robotiq_85_left_finger_tip_link",
    "left_robotiq_85_left_inner_knuckle_link",
    "left_robotiq_85_left_knuckle_link",
    "left_robotiq_85_right_finger_link",
    "left_robotiq_85_right_finger_tip_link",
    "left_robotiq_85_right_inner_knuckle_link",
    "left_robotiq_85_right_knuckle_link",
    "right_robotiq_coupler",
    "right_robotiq_85_base_link",
    "right_robotiq_85_left_finger_link",
    "right_robotiq_85_left_finger_tip_link",
    "right_robotiq_85_right_finger_link",
    "right_robotiq_85_right_finger_tip_link",
    "right_robotiq_85_left_inner_knuckle_link",
    "right_robotiq_85_right_inner_knuckle_link",
    "right_robotiq_85_left_knuckle_link",
    "right_robotiq_85_right_knuckle_link",
]
ARM_LIMITS = {
    "Link1_L_joint": (-3.11, 3.11),
    "Link2_L_joint": (-2.09, 2.09),
    "Link3_L_joint": (-3.11, 3.11),
    "Link4_L_joint": (-2.53, 1.05),
    "Link5_L_joint": (-3.11, 3.11),
    "Link6_L_joint": (-1.05, 1.05),
    "Link7_L_joint": (-1.57, 1.57),
}


class StackBoxes(Node):
    def __init__(self):
        super().__init__("stack_boxes")
        self.declare_parameter("reach", 0.85)
        self.declare_parameter("max_boxes", 0)
        self.declare_parameter("execute", True)
        self.ready_index = None
        self.scene_ids = []
        self.world_boxes = []
        self.joint_state = None
        self.wheel_state = None
        self.arm_seed = None
        self.safe_arm = None
        self.holding = False
        self.retreat_clearance = False
        self.pick_clearance = False
        self.base_pose = None
        self.home_pose = None
        self.steer_hold = [0.0, 0.0, 0.0, 0.0]
        self.suction = self.create_publisher(Bool, "left_suction_cmd", 10)
        self.spawn = self.create_publisher(Int32, "stack_spawn", 10)
        self.steer_pub = self.create_publisher(JointState, "isaac_wheel_steer", 10)
        self.drive_pub = self.create_publisher(JointState, "isaac_wheel_drive", 10)
        self.create_subscription(Int32, "stack_box_ready", self._on_ready, 10)
        self.create_subscription(JointState, "joint_states", self._on_joints, 10)
        self.create_subscription(JointState, "isaac_joint_states", self._on_wheels, 10)
        self.create_subscription(Odometry, "ground_truth_odom", self._on_odom, 10)
        self.move = ActionClient(self, MoveGroup, "move_action")
        self.scene = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.ik = self.create_client(GetPositionIK, "compute_ik")
        self.fk = self.create_client(GetPositionFK, "compute_fk")
        self.validity = self.create_client(GetStateValidity, "check_state_validity")
        self.tf = TransformBroadcaster(self)
        self._tf_logged = False

    def _on_ready(self, msg):
        self.ready_index = int(msg.data)

    def _on_joints(self, msg):
        self.joint_state = msg

    def _on_wheels(self, msg):
        self.wheel_state = msg

    def _on_odom(self, msg):
        quat = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * quat.w * quat.z, 1.0 - 2.0 * quat.z * quat.z)
        self.base_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self._publish_ground_truth_tf()

    def _publish_ground_truth_tf(self):
        """Show Isaac chassis motion in RViz (ground_truth_odom does not publish TF)."""
        if self.base_pose is None:
            return
        x, y, yaw = self.base_pose
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf.sendTransform(transform)
        if not self._tf_logged:
            self._tf_logged = True
            self.get_logger().info(
                "发布 world→base_link TF（来自 ground_truth_odom）。"
                "RViz Fixed Frame 用 world 即可看到底盘移动。"
            )

    def sleep(self, seconds):
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_servers(self):
        if not self.move.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("没有等到 MoveIt 的 move_action。先启动 isaac_moveit.launch.py。")
            return False
        if not self.scene.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("没有等到 apply_planning_scene。")
            return False
        return True

    def _relax_start_tolerance(self):
        """Let execution start when the sampled joints differ by a few degrees.

        The running move_group keeps 0.01 rad until this parameter is set.
        A fresh isaac_moveit launch also reads it from moveit_controllers.yaml.
        """
        client = self.create_client(
            SetParameters, "/moveit_simple_controller_manager/set_parameters"
        )
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "没有改成起点容差。若手臂下降马上报 -4，重启 isaac_moveit.launch.py。"
            )
            self._loosen_tracking()
            return
        request = SetParameters.Request()
        for name, value in (
            ("trajectory_execution.allowed_start_tolerance", 0.05),
            # A short descent is only ~1.5 s on paper. The default 1.2x + 0.5 s
            # cancels it while the cup is still in the air (error -6).
            ("trajectory_execution.allowed_execution_duration_scaling", 4.0),
            ("trajectory_execution.allowed_goal_duration_margin", 3.0),
        ):
            parameter = Parameter()
            parameter.name = name
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            request.parameters.append(parameter)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None or not result.results or not all(item.successful for item in result.results):
            self.get_logger().warn("执行时限没有写进 MoveIt。下降若再报 -6，请重启 isaac_moveit.launch.py。")
        else:
            self.get_logger().info("轨迹起点容差 0.05 rad，执行时限 4 倍再加 3 秒")
        self._loosen_tracking()

    def _loosen_tracking(self):
        """The 0.08 rad / 1.5 s goal aborts a descent while the cup is still in the air.

        Isaac lags the joint command. The action then returns -4 and the arm
        stops on the joint-space arc, beside the box. The tool pose is checked
        after the motion, so this tolerance only has to let the trajectory finish.
        """
        arms = {
            "left_arm_controller": {f"Link{i}_L_joint": 0.15 for i in range(1, 8)},
            "right_arm_controller": {f"Link{i}_R_joint": 0.15 for i in range(1, 8)},
            "torso_controller": {
                "linear_Link": 0.03,
                "waist_pitch_Link": 0.15,
                "waist_yaw_Link": 0.15,
            },
        }
        for controller, goals in arms.items():
            if not self._set_goals(controller, goals, 6.0):
                return
        self.get_logger().info("关节到位容差 0.15 rad，到位等待 6 秒")

    def _set_goals(self, controller, goals, goal_time):
        client = self.create_client(SetParameters, f"/{controller}/set_parameters")
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"没有连上 {controller}，下降仍可能被 1.5 秒到位时间打断。")
            return False
        request = SetParameters.Request()

        def add(name, value):
            parameter = Parameter()
            parameter.name = name
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            request.parameters.append(parameter)

        add("constraints.goal_time", goal_time)
        for joint, goal in goals.items():
            add(f"constraints.{joint}.goal", goal)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None or not result.results or not all(item.successful for item in result.results):
            self.get_logger().warn(f"{controller} 的到位容差没有改成。")
            return False
        return True

    def wait_ready(self, index, timeout):
        end = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.ready_index == index:
                return True
        return False

    def wait_scene(self, timeout):
        """Return the box index Isaac is publishing, whatever it currently is."""
        end = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.ready_index is not None:
                return self.ready_index
        return None

    def set_suction(self, closed):
        message = Bool()
        message.data = bool(closed)
        for _ in range(3):
            self.suction.publish(message)
            self.sleep(0.1)

    def run(self):
        reach = float(self.get_parameter("reach").value)
        max_boxes = int(self.get_parameter("max_boxes").value)
        execute = bool(self.get_parameter("execute").value)
        slots = pallet_slots(reach)
        if max_boxes > 0:
            slots = slots[:max_boxes]
        if not slots:
            self.get_logger().error("可达范围内没有堆叠格。")
            return
        self.get_logger().info(
            f"堆叠 {len(slots)} 只箱子，取料点 {pickup_center()}，托盘中心 {pallet_center()}"
        )
        for slot in slots:
            self.get_logger().info(
                f"格子 {slot.index} 层 {slot.layer} 位置 ({slot.x:.3f}, {slot.y:.3f}, {slot.z:.3f})"
            )
        if not execute:
            return
        if not self.wait_for_servers():
            return
        self._relax_start_tolerance()
        self._load_srdf_pairs()
        if not self._publish_collision_matrix():
            return
        # The suction graph ignores the first sample. Register open before the first close.
        self.set_suction(False)
        # A leftover stack_spawn value is what made Play create box_1 on top of
        # box_0. Publish 0 so the spawner treats the next index as a new command.
        clear = Int32()
        clear.data = 0
        for _ in range(5):
            self.spawn.publish(clear)
            self.sleep(0.05)
        ready = self.wait_scene(20.0)
        if ready is None:
            self.get_logger().error(
                "没有收到 stack_box_ready。先打开 wheel_robot_suction_stack.usd 并保持 Play。"
            )
            return
        if ready > 0:
            self.get_logger().error(
                f"取料点序号已经是 {ready}：架子上的 box_0 还在，上一轮留下的生成指令又叠了一只箱子。"
                "请停止后重新打开 wheel_robot_suction_stack.usd，再按 Play。"
            )
            return
        if not self._wait_base(5.0):
            self.get_logger().error("没有收到 ground_truth_odom。保持场景 Play，底盘才能开到托盘前。")
            return
        self.home_pose = self.base_pose
        self.world_boxes = [
            ("pallet", pallet_center(), pallet_size()),
            ("stand", stand_center(), STAND_SIZE),
            ("pickup", pickup_center(), BOX),
        ]
        if not self._sync_scene():
            self.get_logger().error("规划场景没有写进去。")
            return
        pallet = pallet_center()
        pickup = pickup_center()
        self.get_logger().info(
            f"托盘中心 ({pallet[0]:.3f}, {pallet[1]:.3f}, {pallet[2]:.3f})，"
            f"取料箱心 ({pickup[0]:.3f}, {pickup[1]:.3f}, {pickup[2]:.3f})"
        )
        for place_index, slot in enumerate(slots):
            if not self._pick_and_place(slot):
                self.get_logger().error(f"第 {slot.index} 格失败，停止。取料点不再生成新箱子。")
                return
            next_index = place_index + 1
            if next_index >= len(slots):
                self.get_logger().info("可达格子已经堆完。")
                return
            message = Int32()
            message.data = next_index
            self.spawn.publish(message)
            if not self.wait_ready(next_index, 8.0):
                self.get_logger().error(f"新箱子 {next_index} 没有出现。")
                return
            self.world_boxes.append(("pickup", pickup_center(), BOX))
            self._sync_scene()
            self.sleep(0.4)

    def _pick_and_place(self, slot: Slot):
        pick_z = tool_z_for_pick()
        place_z = tool_z_for_place(slot)
        if not self._drive_base(*self.home_pose):
            return False
        self._sync_scene()
        px, py = self._into_base(*pickup_center()[:2])
        box_top = pickup_box_top()
        self.get_logger().info(
            f"箱顶 {box_top:.3f}，密封面贴住后工具高度 {pick_z:.3f}"
        )
        if not self._move_tool(px, py, pick_z + APPROACH_HEIGHT, "取料上方"):
            return False
        if not self._move_tool(px, py, pick_z, "取料"):
            return False
        # A rising edge is what the suction graph acts on. Repeat it at the
        # contact pose so a missed first sample cannot leave the cup open.
        self.set_suction(False)
        self.sleep(0.3)
        self.set_suction(True)
        self.sleep(1.5)
        self._attach_carried()
        # Leave the stand vertically first. Diagonal carry-out while the box is
        # still on the bench shears the suction seal; Isaac also tracks 2-3 cm
        # behind the command, so a tight FK gate aborts a valid carry.
        clear_z = pick_z + 0.20
        self.pick_clearance = True
        self._publish_collision_matrix()
        try:
            if not self._move_tool(px, py, clear_z, "抬起", carry_tolerance=0.05):
                return False
            measured = self._measured_tool()
            if measured is not None and measured.position.z < pick_z + 0.10:
                self.get_logger().warn("抬起高度不够，再抬高一节")
                if not self._move_tool(px, py, clear_z + 0.06, "再抬起", carry_tolerance=0.05):
                    return False
                measured = self._measured_tool()
            carry_z = clear_z
            if measured is not None:
                carry_z = max(carry_z, measured.position.z)
            if not self._move_carry_xy(px + 0.10, py + 0.03, carry_z, "移出"):
                return False
            if not self._move_tool(
                px + 0.10,
                py + 0.03,
                pick_z + APPROACH_HEIGHT + 0.05,
                "行车高度",
                carry_tolerance=0.06,
            ):
                return False
        finally:
            self.pick_clearance = False
            self._publish_collision_matrix()
        self.safe_arm = dict(self.arm_seed) if self.arm_seed else None
        park = base_for_slot(slot)
        self.get_logger().info(
            f"开到底盘 ({park[0]:.3f}, {park[1]:.3f}) 再放格子 {slot.index}"
        )
        if not self._drive_base(*park):
            return False
        self._sync_scene()
        sx, sy = self._into_base(slot.x, slot.y)
        if not self._move_tool(sx, sy, place_z + APPROACH_HEIGHT, "放置上方"):
            return False
        if not self._move_tool(sx, sy, place_z, "放置"):
            return False
        self.set_suction(False)
        self.sleep(0.6)
        settled = (slot.x, slot.y, slot.support_z + BOX[2] / 2.0)
        self._detach_at(slot.index, settled)
        if not self._retreat(sx, sy, place_z + APPROACH_HEIGHT):
            self.get_logger().error("撤离失败，手臂还停在托盘里，停止。")
            return False
        self.get_logger().info(f"已放下格子 {slot.index}")
        return True

    def _retreat(self, x, y, z):
        if self._move_tool(x, y, z, "撤离"):
            return True
        self.get_logger().warn("垂直撤离被托盘挡住，允许手臂贴着托盘再抬一次")
        self.retreat_clearance = True
        self._publish_collision_matrix()
        try:
            if self._move_tool(x, y, z, "撤离"):
                return True
            if self.safe_arm and self._move_named_joints(self.safe_arm):
                self.get_logger().info("手臂已回到搬运姿态")
                return True
        finally:
            self.retreat_clearance = False
            self._publish_collision_matrix()
        return False

    def _move_named_joints(self, joints):
        constraints = Constraints()
        for name, value in joints.items():
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.05
            joint.tolerance_below = 0.05
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return self._move("left_arm", constraints)

    def _wait_base(self, timeout):
        end = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.get_clock().now() < end:
            if self.base_pose is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def _into_base(self, x, y):
        base_x, base_y, yaw = self.base_pose
        dx = x - base_x
        dy = y - base_y
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return cosine * dx + sine * dy, -sine * dx + cosine * dy

    def _sync_scene(self):
        objects = []
        self.scene_ids = []
        for name, center, size in self.world_boxes:
            local_x, local_y = self._into_base(center[0], center[1])
            objects.append(self._box(name, (local_x, local_y, center[2]), size))
            self.scene_ids.append(name)
        return self._apply_scene(objects)

    def _measured_steer(self):
        if self.wheel_state is None:
            return None
        positions = {}
        for name, value in zip(self.wheel_state.name, self.wheel_state.position):
            positions[name] = float(value)
            positions[name.rsplit("/", 1)[-1]] = float(value)
        angles = []
        for steer_name, _, _, _, _ in MODULES:
            if steer_name not in positions:
                return None
            angles.append(positions[steer_name])
        return angles

    def _send_wheels(self, angles, drives):
        stamp = self.get_clock().now().to_msg()
        steer = JointState()
        steer.header.stamp = stamp
        steer.name = [module[0] for module in MODULES]
        steer.position = [float(value) for value in angles]
        drive = JointState()
        drive.header.stamp = stamp
        drive.name = [module[1] for module in MODULES]
        drive.velocity = [float(value) for value in drives]
        self.steer_pub.publish(steer)
        self.drive_pub.publish(drive)

    def _stop_wheels(self):
        for _ in range(5):
            self._send_wheels(self.steer_hold, [0.0, 0.0, 0.0, 0.0])
            self.sleep(0.05)

    def _drive_base(self, x, y, yaw):
        """Holonomic drive on the swerve base. MoveIt's world frame is the chassis."""
        deadline = self.get_clock().now() + Duration(seconds=45.0)
        settled = 0
        hold = list(self.steer_hold)
        last_report = self.get_clock().now()
        scene_tick = 0
        pos_tol = 0.04
        yaw_tol = 0.06
        while rclpy.ok() and self.get_clock().now() < deadline:
            if self.base_pose is None:
                self.sleep(0.05)
                continue
            base_x, base_y, base_yaw = self.base_pose
            error_x = x - base_x
            error_y = y - base_y
            error_yaw = math.atan2(math.sin(yaw - base_yaw), math.cos(yaw - base_yaw))
            dist = math.hypot(error_x, error_y)
            if dist < pos_tol and abs(error_yaw) < yaw_tol:
                settled += 1
                self._send_wheels(hold, [0.0, 0.0, 0.0, 0.0])
                if settled >= 5:
                    self._stop_wheels()
                    self.get_logger().info(
                        f"底盘到位 ({base_x:.3f}, {base_y:.3f}) 偏航 {base_yaw:.3f}，"
                        f"目标 ({x:.3f}, {y:.3f})"
                    )
                    return True
                self.sleep(0.05)
                continue
            settled = 0
            cosine = math.cos(base_yaw)
            sine = math.sin(base_yaw)
            forward = cosine * error_x + sine * error_y
            lateral = -sine * error_x + cosine * error_y
            speed = math.hypot(forward, lateral)
            limit = 0.18
            if speed > limit:
                forward *= limit / speed
                lateral *= limit / speed
            elif dist < 0.08 and speed > 1.0e-6:
                # The last few centimetres need a minimum creep. Otherwise the
                # steer-alignment gate keeps drives at zero and the log sits at
                # ~0.036 m forever.
                creep = 0.06
                forward *= creep / speed
                lateral *= creep / speed
                speed = creep
            turn = 0.0 if dist < 0.08 else max(-0.35, min(0.35, 1.1 * error_yaw))
            # Keep the steer target chosen for this twist. Re-seeding it from
            # the measured angle every cycle flips a wheel between the two
            # equivalent headings, so the drive speed stays at zero.
            measured = self._measured_steer()
            angles, drives = twist_to_wheels(0.8 * forward, 0.8 * lateral, turn, hold)
            hold = angles
            self.steer_hold = angles
            aligned = measured is not None and all(
                steer_error(current, target) <= 0.12 for current, target in zip(measured, angles)
            )
            if dist < 0.06:
                aligned = True
            if not aligned:
                drives = [0.0, 0.0, 0.0, 0.0]
            now = self.get_clock().now()
            if (now - last_report).nanoseconds > 5.0e9:
                last_report = now
                self.get_logger().info(
                    f"底盘 ({base_x:.3f}, {base_y:.3f}) 偏航 {base_yaw:.3f}，"
                    f"距目标 {dist:.3f} m"
                    + ("" if aligned else "，转向还没对齐")
                )
            scene_tick += 1
            if scene_tick >= 10:
                scene_tick = 0
                self._sync_scene()
            self._send_wheels(angles, drives)
            self.sleep(0.05)
        self._stop_wheels()
        if self.base_pose is None:
            self.get_logger().error(f"底盘没有开到 ({x:.3f}, {y:.3f})，没有里程计")
        else:
            base_x, base_y, base_yaw = self.base_pose
            self.get_logger().error(
                f"底盘没有开到 ({x:.3f}, {y:.3f})，停在 ({base_x:.3f}, {base_y:.3f}) "
                f"偏航 {base_yaw:.3f}"
            )
        return False

    def _joint_position(self, name):
        if self.joint_state is None:
            return None
        positions = dict(zip(self.joint_state.name, self.joint_state.position))
        if name not in positions:
            return None
        return float(positions[name])

    def _move_torso(self, lift):
        # A goal that is already satisfied produces an empty trajectory, which
        # MoveIt reports as INVALID_MOTION_PLAN (-2).
        targets = (
            ("linear_Link", lift),
            ("waist_pitch_Link", 0.0),
            ("waist_yaw_Link", 0.0),
        )
        already_there = True
        for name, value in targets:
            current = self._joint_position(name)
            if current is None or abs(current - value) >= 0.03:
                already_there = False
                break
        if already_there:
            return True
        constraints = Constraints()
        for name, value in (
            ("linear_Link", lift),
            ("waist_pitch_Link", 0.0),
            ("waist_yaw_Link", 0.0),
        ):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.02
            joint.tolerance_below = 0.02
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        if self._move("torso", constraints):
            return True
        # An empty trajectory is reported as -2. If the joints are already
        # on the goal, the lift does not need to move.
        if self._torso_matches(targets, 0.05):
            self.get_logger().info(f"升降已在 {lift:.3f}，空规划按到位处理")
            return True
        measured = ", ".join(
            f"{name}={self._joint_position(name)}" for name, _value in targets
        )
        self.get_logger().error(f"升降目标 {lift:.3f}，当前 {measured}")
        return False

    def _torso_matches(self, targets, tolerance):
        for name, value in targets:
            current = self._joint_position(name)
            if current is None or abs(current - value) >= tolerance:
                return False
        return True

    def _current_arm(self):
        if self.joint_state is None:
            return None
        positions = dict(zip(self.joint_state.name, self.joint_state.position))
        if any(name not in positions for name in ARM_LIMITS):
            return None
        return {name: float(positions[name]) for name in ARM_LIMITS}

    def _lift_now(self):
        if self.joint_state is None:
            return 0.0
        positions = dict(zip(self.joint_state.name, self.joint_state.position))
        return float(positions.get("linear_Link", 0.0))

    def _candidate_lifts(self, tool_z):
        preferred = lift_for_tool_z(tool_z)
        lifts = []
        for value in (preferred, self._lift_now(), 0.0, 0.2, 0.4, 0.6):
            value = min(LIFT_LIMIT, max(0.0, float(value)))
            if all(abs(value - other) > 0.03 for other in lifts):
                lifts.append(value)
        return lifts

    def _ik_seed(self, arm, lift):
        names = []
        positions = []
        if self.joint_state is not None:
            names = list(self.joint_state.name)
            positions = [float(value) for value in self.joint_state.position]
        index = {name: i for i, name in enumerate(names)}

        def set_joint(name, value):
            if name in index:
                positions[index[name]] = float(value)
                return
            index[name] = len(names)
            names.append(name)
            positions.append(float(value))

        for name, value in arm.items():
            set_joint(name, value)
        set_joint("linear_Link", lift)
        set_joint("waist_pitch_Link", 0.0)
        set_joint("waist_yaw_Link", 0.0)
        state = JointState()
        state.name = names
        state.position = positions
        return state

    def _solve_arm(self, x, y, z, max_joint_delta=None):
        """Pick a collision-free left-arm solution for a downward tool pose.

        A pose constraint makes MoveIt seed inverse kinematics from the current
        joints. That seed folds Link7 into Link5, so every goal sample is
        rejected even though other solutions are clear.
        """
        if not self.ik.service_is_ready() or not self.validity.service_is_ready():
            return None
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        # Rx(180). w must be set; the message default is 1, and (1,0,0,1)
        # normalizes to Rx(90), which leaves the seal facing sideways.
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        seeds = []
        if self.arm_seed is not None:
            seeds.append(self.arm_seed)
        current = self._current_arm()
        if current is not None:
            seeds.append(current)
        for _ in range(6):
            seeds.append({name: random.uniform(low, high) for name, (low, high) in ARM_LIMITS.items()})
        reference = self.arm_seed or current or {name: 0.0 for name in ARM_LIMITS}
        preferred = lift_for_tool_z(z)
        # Stay at the current lift for the vertical approach and retreat.
        # Changing it between the pick and the lift-off is what produced the
        # empty torso plan.
        current_lift = min(LIFT_LIMIT, max(0.0, self._lift_now()))
        lifts = [current_lift]
        for value in self._candidate_lifts(z):
            if all(abs(value - other) > 0.03 for other in lifts):
                lifts.append(value)
        for lift_index, lift in enumerate(lifts):
            arm_seeds = seeds if lift_index == 0 else seeds[:3]
            best = None
            best_cost = None
            for seed in arm_seeds:
                request = GetPositionIK.Request()
                request.ik_request.group_name = "left_arm"
                request.ik_request.ik_link_name = "tool0_L"
                request.ik_request.pose_stamped = pose
                request.ik_request.avoid_collisions = True
                request.ik_request.timeout.sec = 0
                request.ik_request.timeout.nanosec = 200000000
                request.ik_request.robot_state = RobotState()
                request.ik_request.robot_state.joint_state = self._ik_seed(seed, lift)
                future = self.ik.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                result = future.result()
                if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
                    continue
                check = GetStateValidity.Request()
                check.robot_state = result.solution
                check.group_name = "left_arm"
                checked = self.validity.call_async(check)
                rclpy.spin_until_future_complete(self, checked)
                validity = checked.result()
                if validity is None or not validity.valid:
                    continue
                if not self._tool_reaches(result.solution, x, y, z):
                    continue
                positions = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
                if any(name not in positions for name in ARM_LIMITS):
                    continue
                solution = {name: float(positions[name]) for name in ARM_LIMITS}
                solution["linear_Link"] = float(lift)
                solution["waist_pitch_Link"] = 0.0
                solution["waist_yaw_Link"] = 0.0
                # A short descent must stay on the current elbow. A distant
                # solution swings the cup off the box and the loose controller
                # goal then reports success anyway.
                if max_joint_delta is not None and current is not None:
                    if any(
                        abs(solution[name] - current[name]) > max_joint_delta for name in ARM_LIMITS
                    ):
                        continue
                cost = sum(abs(solution[name] - reference[name]) for name in ARM_LIMITS)
                cost += 3.0 * abs(solution["linear_Link"] - preferred)
                if best_cost is None or cost < best_cost:
                    best = solution
                    best_cost = cost
            if best is not None:
                return best
        return None

    def _tool_reaches(self, robot_state, x, y, z):
        pose = self._forward_tool(robot_state)
        if pose is None:
            return False
        dx = pose.position.x - x
        dy = pose.position.y - y
        dz = pose.position.z - z
        if dx * dx + dy * dy + dz * dz > 0.02 * 0.02:
            return False
        quat = pose.orientation
        down = 1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)
        return down < -0.9

    def _forward_tool(self, robot_state):
        if not self.fk.service_is_ready():
            return None
        request = GetPositionFK.Request()
        request.header.frame_id = "world"
        request.fk_link_names = ["tool0_L"]
        request.robot_state = robot_state
        future = self.fk.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS or not result.pose_stamped:
            return None
        return result.pose_stamped[0].pose

    def _measured_tool(self):
        if self.joint_state is None:
            return None
        state = RobotState()
        state.joint_state = self.joint_state
        return self._forward_tool(state)

    def _wait_tool(self, x, y, z, timeout, tolerance=0.035):
        """Isaac often finishes the last centimetres after the controller says done."""
        end = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.get_clock().now() < end:
            if self._at_tool(x, y, z, tolerance) or self._measured_tool() is None:
                return True
            self.sleep(0.1)
        return False

    def _at_tool(self, x, y, z, tolerance=0.035):
        pose = self._measured_tool()
        if pose is None:
            return False
        dx = pose.position.x - x
        dy = pose.position.y - y
        dz = pose.position.z - z
        return dx * dx + dy * dy + dz * dz <= tolerance * tolerance

    def _log_tool(self, label, x, y, z):
        pose = self._measured_tool()
        if pose is None:
            self.get_logger().warn(f"{label} 没有关节反馈，不能确认工具位置")
            return
        self.get_logger().info(
            f"{label} 实测 ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})，"
            f"目标 ({x:.3f}, {y:.3f}, {z:.3f})"
        )

    def _move_carry_xy(self, x, y, z, label, step=0.05):
        """Slide the carried box sideways at a fixed height."""
        pose = self._measured_tool()
        if pose is None:
            return False
        cx, cy = pose.position.x, pose.position.y
        distance = math.hypot(x - cx, y - cy)
        if distance < 0.02:
            return True
        count = max(1, int(math.ceil(distance / step)))
        for index in range(1, count + 1):
            fraction = index / count
            tx = cx + (x - cx) * fraction
            ty = cy + (y - cy) * fraction
            if not self._move_tool_direct(tx, ty, z, label, 0.05, False):
                return False
            if not self._wait_tool(tx, ty, z, 3.0, 0.05):
                if not self._at_tool(tx, ty, z, 0.06):
                    self._log_tool(label, tx, ty, z)
                    return False
        return True

    def _move_cartesian(self, x, y, z, label, step=0.04):
        """Walk the cup in short straight segments so it can leave the chest."""
        pose = self._measured_tool()
        if pose is None:
            return self._move_tool_direct(x, y, z, label, 0.025, False)
        cx, cy, cz = pose.position.x, pose.position.y, pose.position.z
        distance = math.sqrt((x - cx) ** 2 + (y - cy) ** 2 + (z - cz) ** 2)
        count = max(1, int(math.ceil(distance / step)))
        for index in range(1, count + 1):
            fraction = index / count
            if not self._move_tool_direct(
                cx + (x - cx) * fraction,
                cy + (y - cy) * fraction,
                cz + (z - cz) * fraction,
                label,
                0.025,
                False,
            ):
                return False
        return True

    def _move_tool(self, x, y, z, label, carry_tolerance=None):
        # A single joint-space plan from the hover pose down to the box swings
        # out to the side. The controller then aborts on that arc (-4) and the
        # cup never meets the cardboard. Short vertical steps stay over the box.
        if carry_tolerance is not None:
            return self._move_tool_direct(
                x, y, z, label, carry_tolerance, False, wait=4.0
            )
        if self._mostly_vertical(x, y, z):
            return self._move_vertical(x, y, z, label)
        return self._move_tool_direct(x, y, z, label)

    def _mostly_vertical(self, x, y, z):
        pose = self._measured_tool()
        if pose is None:
            return False
        horizontal = math.hypot(x - pose.position.x, y - pose.position.y)
        return horizontal <= 0.06 and abs(z - pose.position.z) > 0.05

    def _move_vertical(self, x, y, z, label):
        pose = self._measured_tool()
        cursor = pose.position.z
        direction = 1.0 if z > cursor else -1.0
        # The forearm meets the pickup box on the way down. Allow that contact
        # so inverse kinematics can keep the same elbow instead of swinging aside.
        self.pick_clearance = True
        self._publish_collision_matrix()
        try:
            while True:
                remaining = z - cursor
                if abs(remaining) <= 0.045:
                    target_z = z
                else:
                    target_z = cursor + direction * 0.04
                # The last downward step has to put the lip on the cardboard.
                # 2.5 cm still allows the cup to stop on the surface instead of
                # sinking the full press into the box.
                tolerance = 0.025 if direction < 0 and abs(target_z - z) <= 1e-4 else 0.035
                if not self._move_tool_direct(x, y, target_z, label, tolerance, False):
                    return False
                cursor = target_z
                if abs(cursor - z) <= 1e-4:
                    return True
        finally:
            self.pick_clearance = False
            self._publish_collision_matrix()

    def _move_tool_direct(
        self, x, y, z, label, tolerance=0.035, allow_pose_fallback=True, wait=2.0
    ):
        measured = self._measured_tool()
        max_joint_delta = None
        if measured is not None:
            gap = math.sqrt(
                (measured.position.x - x) ** 2
                + (measured.position.y - y) ** 2
                + (measured.position.z - z) ** 2
            )
            if gap < 0.08:
                max_joint_delta = 0.45
        solved = self._solve_arm(x, y, z, max_joint_delta)
        if solved is None and max_joint_delta is not None:
            self.get_logger().warn(f"{label} 近处逆解不够，放宽关节步长")
            solved = self._solve_arm(x, y, z, max_joint_delta * 2.0)
        if solved is not None:
            arm_only = {name: solved[name] for name in ARM_LIMITS}
            self.arm_seed = arm_only
            if not self._move_torso(solved["linear_Link"]):
                return False
            constraints = Constraints()
            for name, value in arm_only.items():
                joint = JointConstraint()
                joint.joint_name = name
                joint.position = float(value)
                joint.tolerance_above = 0.02
                joint.tolerance_below = 0.02
                joint.weight = 1.0
                constraints.joint_constraints.append(joint)
            self.get_logger().info(
                f"{label} ({x:.3f}, {y:.3f}, {z:.3f}) 升降 {solved['linear_Link']:.3f}"
            )
            if self._arm_at(arm_only, 0.02) and self._at_tool(x, y, z, tolerance):
                self._log_tool(label, x, y, z)
                return True
            # -2 is an empty plan. Accept it only when the measured tool is
            # already on the target; otherwise the arm never left the last pose.
            if self._move("left_arm", constraints) or self._at_tool(x, y, z, tolerance):
                if self._wait_tool(x, y, z, wait, tolerance):
                    self._log_tool(label, x, y, z)
                    return True
                # Carrying: accept a few centimetres of lag if the box is clearly up.
                if self.holding and self._at_tool(x, y, z, tolerance + 0.02):
                    self._log_tool(label, x, y, z)
                    self.get_logger().warn(f"{label} 跟踪略慢，按到位继续")
                    return True
            self._log_tool(label, x, y, z)
            self.get_logger().error(f"{label} 手臂没有到达目标")
            return False
        if not allow_pose_fallback:
            self.get_logger().error(f"{label} 没有贴近当前姿态的逆解，吸盘不能横着甩开")
            return False
        if not self._move_torso(lift_for_tool_z(z)):
            return False
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        # Rx(180): tool +Z, the seal axis, points down.
        pose.orientation.x = 1.0
        pose.orientation.w = 0.0
        position = PositionConstraint()
        position.header.frame_id = "world"
        position.link_name = "tool0_L"
        position.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        position.constraint_region.primitives = [sphere]
        region_pose = Pose()
        region_pose.position = pose.position
        region_pose.orientation.w = 1.0
        position.constraint_region.primitive_poses = [region_pose]
        orientation = OrientationConstraint()
        orientation.header.frame_id = "world"
        orientation.link_name = "tool0_L"
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = 0.35
        orientation.absolute_y_axis_tolerance = 0.35
        orientation.absolute_z_axis_tolerance = 1.5
        orientation.weight = 1.0
        constraints = Constraints()
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        self.get_logger().warn(f"{label} 没有找到无碰撞逆解，改用位姿约束")
        self.get_logger().info(f"{label} ({x:.3f}, {y:.3f}, {z:.3f})")
        return self._move("left_arm", constraints)

    def _arm_at(self, joints, tolerance):
        for name, value in joints.items():
            current = self._joint_position(name)
            if current is None or abs(current - value) > tolerance:
                return False
        return True

    def _move(self, group, constraints):
        for attempt in range(3):
            code = self._move_once(group, constraints)
            if code == MoveItErrorCodes.SUCCESS:
                return True
            # -4 is CONTROL_FAILED: the trajectory was dropped before the arm moved,
            # usually because its first point was outside the 0.01 rad start tolerance.
            if code == MoveItErrorCodes.INVALID_MOTION_PLAN:
                self.get_logger().warn(f"{group} 路径无效，错误码 -2")
                return False
            retry = code in (MoveItErrorCodes.CONTROL_FAILED, MoveItErrorCodes.TIMED_OUT)
            if not retry or attempt == 2:
                self.get_logger().error(f"{group} 规划或执行失败，错误码 {code}")
                return False
            if code == MoveItErrorCodes.TIMED_OUT:
                self.get_logger().warn(f"{group} 执行超时，从当前位置再走")
            else:
                self.get_logger().warn(f"{group} 轨迹被控制器拒绝，重试")
            self.sleep(0.4)
        return False

    def _move_once(self, group, constraints):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = group
        request.num_planning_attempts = 10
        request.allowed_planning_time = 8.0
        request.max_velocity_scaling_factor = 0.15
        request.max_acceleration_scaling_factor = 0.15
        request.planner_id = "RRTConnectkConfigDefault"
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        request.workspace_parameters.header.frame_id = "world"
        request.workspace_parameters.min_corner = Vector3(x=-1.5, y=-1.5, z=-0.2)
        request.workspace_parameters.max_corner = Vector3(x=2.0, y=2.5, z=2.2)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        send = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{group} 目标被拒绝")
            return MoveItErrorCodes.FAILURE
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            return MoveItErrorCodes.FAILURE
        return wrapped.result.error_code.val

    def _box(self, object_id, center, size):
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        pose = Pose()
        pose.position.x = float(center[0])
        pose.position.y = float(center[1])
        pose.position.z = float(center[2])
        pose.orientation.w = 1.0
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def _load_srdf_pairs(self):
        srdf = get_package_share_directory("wheel_robot_moveit_config") + "/config/wheel_robot.srdf"
        self._srdf_allowed = set()
        self._srdf_names = []
        for tag in ET.parse(srdf).getroot().findall("disable_collisions"):
            link1 = tag.attrib["link1"]
            link2 = tag.attrib["link2"]
            self._srdf_allowed.add((link1, link2))
            self._srdf_allowed.add((link2, link1))
            if link1 not in self._srdf_names:
                self._srdf_names.append(link1)
            if link2 not in self._srdf_names:
                self._srdf_names.append(link2)

    def _publish_collision_matrix(self):
        """Replace the allowed-collision matrix with the SRDF pairs plus the cup.

        MoveIt stores this matrix as a whole. A later message that names only the
        wrist and a box wipes the adjacent-link pairs, and every plan then starts
        in self-collision.
        """
        links = EE_LINKS + GRIPPER_LINKS
        if self.retreat_clearance or self.pick_clearance:
            links = links + ["Link1_L", "Link2_L", "Link3_L", "Link4_L"]
        names = list(self._srdf_names)
        extra = ["carried"] if self.holding else []
        for name in links + list(self.scene_ids) + extra:
            if name not in names:
                names.append(name)
        matrix = AllowedCollisionMatrix()
        matrix.entry_names = names
        for name in names:
            entry = AllowedCollisionEntry()
            enabled = []
            for other in names:
                adjacent = (name, other) in self._srdf_allowed
                cup_on_object = (name in links and other in self.scene_ids) or (
                    other in links and name in self.scene_ids
                )
                # The grasped box starts sitting on the stand. That contact is
                # the pick, not a collision that should cancel the lift.
                held = (name == "carried" and other in self.scene_ids + links) or (
                    other == "carried" and name in self.scene_ids + links
                )
                enabled.append(adjacent or cup_on_object or held)
            entry.enabled = enabled
            matrix.entry_values.append(entry)
        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = matrix
        if not self._call_scene(scene):
            self.get_logger().error("碰撞矩阵没有写进去。请重启 isaac_moveit.launch.py 后再跑。")
            return False
        return True

    def _apply_scene(self, objects):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = list(objects)
        if not self._call_scene(scene):
            self.get_logger().error("apply_planning_scene 返回失败")
            return False
        return self._publish_collision_matrix()

    def _call_scene(self, scene):
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.scene.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        return result is not None and result.success

    def _attach_carried(self):
        remove = self._box("pickup", pickup_center(), BOX)
        remove.operation = CollisionObject.REMOVE
        carried = CollisionObject()
        carried.header.frame_id = "tool0_L"
        carried.id = "carried"
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [BOX[0], BOX[1], BOX[2]]
        pose = Pose()
        # Box center is along tool +Z. Rx(180) keeps the box upright in the world.
        pose.position.z = carried_offset()
        pose.orientation.x = 1.0
        pose.orientation.w = 0.0
        carried.primitives = [primitive]
        carried.primitive_poses = [pose]
        carried.operation = CollisionObject.ADD
        attached = AttachedCollisionObject()
        attached.link_name = "tool0_L"
        attached.object = carried
        attached.touch_links = ["tool0_L", "Link7_L", "Link6_L", "Link5_L"] + GRIPPER_LINKS
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [remove]
        scene.robot_state.attached_collision_objects = [attached]
        scene.robot_state.is_diff = True
        if not self._call_scene(scene):
            return
        self.holding = True
        self.world_boxes = [item for item in self.world_boxes if item[0] != "pickup"]
        if "pickup" in self.scene_ids:
            self.scene_ids.remove("pickup")
        self._publish_collision_matrix()

    def _detach_at(self, index, center):
        remove = CollisionObject()
        remove.id = "carried"
        remove.operation = CollisionObject.REMOVE
        attached = AttachedCollisionObject()
        attached.link_name = "tool0_L"
        attached.object = remove
        placed_id = f"placed_{index}"
        self.world_boxes.append((placed_id, center, BOX))
        self.scene_ids.append(placed_id)
        local_x, local_y = self._into_base(center[0], center[1])
        placed = self._box(placed_id, (local_x, local_y, center[2]), BOX)
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [placed]
        scene.robot_state.attached_collision_objects = [attached]
        scene.robot_state.is_diff = True
        if not self._call_scene(scene):
            return
        self.holding = False
        self._publish_collision_matrix()


def main():
    rclpy.init()
    node = StackBoxes()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
