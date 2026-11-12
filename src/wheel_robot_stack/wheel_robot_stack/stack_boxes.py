"""Pick from one fixed station and place onto the pallet pattern.

Isaac Sim plays ``wheel_robot_suction_stack_riser.usda``. The node plans with
MoveIt. It picks one box, waits for the next box at the same stand, picks
that one with the other hand, drives to the pallet, and sets the two down
one after the other.
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
    PlanningSceneComponents,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity,
)
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from wheel_robot_nav.kinematics import MODULES, steer_error, twist_to_wheels
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Int32, String

from wheel_robot_stack.pattern import (
    APPROACH_HEIGHT,
    BOX,
    LIFT_LIMIT,
    STAND_SIZE,
    Slot,
    carry_pairs,
    carried_offset,
    lift_for_tool_z,
    pallet_center,
    pallet_size,
    pallet_slots,
    pick_plan,
    pickup_box_top,
    unreachable_slots,
    pickup_center,
    route_around,
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
ARM_LIMITS_R = {name.replace("_L_", "_R_"): bounds for name, bounds in ARM_LIMITS.items()}


class _Arm:
    """One suction arm: its joints, cup, and the box it is currently holding."""

    def __init__(self, name, limits):
        tag = "L" if name == "left" else "R"
        self.name = name
        self.limits = limits
        self.group = f"{name}_arm"
        self.tool = f"tool0_{tag}"
        self.ee = [f"Link5_{tag}", f"Link6_{tag}", f"Link7_{tag}", self.tool]
        self.upper = [f"Link{index}_{tag}" for index in range(1, 5)]
        self.touch = [self.tool, f"Link7_{tag}", f"Link6_{tag}", f"Link5_{tag}"]
        self.carried = f"carried_{name}"
        self.seed = None
        self.holding = False
        self.grip = None
        self.publisher = None
        self.trajectory = None


class StackBoxes(Node):
    def __init__(self):
        super().__init__("stack_boxes")
        self.declare_parameter("reach", 0.40)
        self.declare_parameter("max_boxes", 0)
        self.declare_parameter("execute", True)
        self.ready_index = None
        self.scene_ids = []
        self.world_boxes = []
        self.joint_state = None
        self.wheel_state = None
        self.box_poses = {}
        self.place_path = []
        self.pick_clearance = False
        self.place_supports = set()
        self.base_pose = None
        self.home_pose = None
        self.steer_hold = [0.0, 0.0, 0.0, 0.0]
        self.side = "left"
        self.waist_yaw = 0.0
        self._arms = {
            "left": _Arm("left", ARM_LIMITS),
            "right": _Arm("right", ARM_LIMITS_R),
        }
        for name, topic in (("left", "left_suction_cmd"), ("right", "right_suction_cmd")):
            self._arms[name].publisher = self.create_publisher(Bool, topic, 10)
            self._arms[name].trajectory = ActionClient(
                self,
                FollowJointTrajectory,
                f"/{name}_arm_controller/follow_joint_trajectory",
            )
        self.spawn = self.create_publisher(Int32, "stack_spawn", 10)
        self.steer_pub = self.create_publisher(JointState, "isaac_wheel_steer", 10)
        self.drive_pub = self.create_publisher(JointState, "isaac_wheel_drive", 10)
        self.create_subscription(Int32, "stack_box_ready", self._on_ready, 10)
        self.create_subscription(Bool, "left_suction_holding", lambda msg: self._on_grip("left", msg), 10)
        self.create_subscription(Bool, "right_suction_holding", lambda msg: self._on_grip("right", msg), 10)
        self.create_subscription(String, "stack_box_poses", self._on_box_poses, 10)
        self.create_subscription(JointState, "joint_states", self._on_joints, 10)
        self.create_subscription(JointState, "isaac_joint_states", self._on_wheels, 10)
        self.create_subscription(Odometry, "ground_truth_odom", self._on_odom, 10)
        self.move = ActionClient(self, MoveGroup, "move_action")
        self.torso_traj = ActionClient(
            self, FollowJointTrajectory, "/torso_controller/follow_joint_trajectory"
        )
        self.scene = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.get_scene = self.create_client(GetPlanningScene, "get_planning_scene")
        self.ik = self.create_client(GetPositionIK, "compute_ik")
        self.fk = self.create_client(GetPositionFK, "compute_fk")
        self.validity = self.create_client(GetStateValidity, "check_state_validity")
        self.tf = TransformBroadcaster(self)
        self._tf_logged = False

    def _arm(self):
        return self._arms[self.side]

    def _use(self, name):
        self.side = name
        return self._arm()

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
        for arm in self._arms.values():
            if not arm.trajectory.wait_for_server(timeout_sec=30.0):
                self.get_logger().error(f"没有等到 {arm.name} 手臂轨迹。先等控制器起来再放箱子。")
                return False
        if not self.torso_traj.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("没有等到升降轨迹。先等控制器起来再放箱子。")
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

    def _on_grip(self, name, msg):
        self._arms[name].grip = bool(msg.data)

    def _on_box_poses(self, msg):
        poses = {}
        for part in msg.data.split(";"):
            if ":" not in part:
                continue
            index, xyz = part.split(":", 1)
            values = xyz.split(",")
            if len(values) not in (3, 7):
                continue
            try:
                poses[int(index)] = tuple(float(value) for value in values)
            except ValueError:
                continue
        if poses:
            self.box_poses = poses

    def _apply_measured_boxes(self):
        """Replace planned centers with the prim centers Isaac is simulating."""
        if not self.box_poses:
            return
        updated = []
        for name, center, size in self.world_boxes:
            measured = None
            if name.startswith("placed_"):
                measured = self.box_poses.get(int(name.split("_", 1)[1]))
            if measured is not None:
                position = measured[:3]
                shift = math.dist(position, center[:3])
                if shift > 0.01:
                    self.get_logger().info(
                        f"{name} 实测 ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})，"
                        f"比规划偏了 {shift * 100:.1f} cm"
                    )
                center = position
            updated.append((name, center, size))
        self.world_boxes = updated

    def _measured_support(self, slot):
        """Placed box directly under this slot, from the simulated prim pose."""
        best = None
        best_distance = 0.16
        for name, center, _size in self.world_boxes:
            if not name.startswith("placed_"):
                continue
            if center[2] > slot.z - 0.03:
                continue
            distance = math.hypot(center[0] - slot.x, center[1] - slot.y)
            if distance < best_distance:
                best = center
                best_distance = distance
        return best

    def set_suction(self, closed):
        message = Bool()
        message.data = bool(closed)
        publisher = self._arm().publisher
        for _ in range(3):
            publisher.publish(message)
            self.sleep(0.1)

    def _wait_holding(self, timeout=1.8, stable=0.4):
        """True when the cup reports a real grip, False if it stays open.

        None means Isaac never published left_suction_holding. The command
        topic alone cannot tell a missed seal from a grip.
        """
        self._arm().grip = None
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        since = None
        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._arm().grip is True:
                if since is None:
                    since = self.get_clock().now()
                elif self.get_clock().now() - since >= Duration(seconds=stable):
                    return True
            else:
                since = None
            self.sleep(0.05)
        if self._arm().grip is None:
            return None
        return False

    def _pulse_suction(self):
        # The graph ignores a repeated True. Open, then close, at contact.
        self.set_suction(False)
        self.sleep(0.3)
        self.set_suction(True)

    def _grasp(self, x, y, z):
        """Close on the box. A miss is another approach, never an empty carry."""
        for attempt in range(3):
            if attempt:
                self.get_logger().warn(f"吸盘没有吸住纸箱，第 {attempt + 1} 次再吸取")
                self.set_suction(False)
                if not self._move_tool(x, y, z + 0.04, "再吸取上方"):
                    return False
                if not self._move_tool(x, y, z, "再吸取"):
                    return False
            self._pulse_suction()
            held = self._wait_holding()
            if held is True:
                self.get_logger().info("吸盘已吸住纸箱")
                return True
            if held is None:
                self.get_logger().error(
                    "没有收到吸盘是否吸住的状态。请重新打开 wheel_robot_suction_stack_riser.usda 后再放。"
                )
                self.set_suction(False)
                return False
        self.get_logger().error("三次都没有吸住，这一格不放置，避免空放后下一箱叠空。")
        self.set_suction(False)
        return False

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
        skipped = unreachable_slots(reach)
        if skipped:
            self.get_logger().warn(
                "托盘正中每层有一格。底盘停在托盘外时，左手和右手的肩膀离这一格都约 0.64 m，"
                "吸盘朝下时手臂够不着。这一格跳过，其余格子两只手一起从下往上堆。"
            )
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
        for name in ("left", "right"):
            self._use(name)
            self.set_suction(False)
        self._use("left")
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
        pairs = carry_pairs(slots, reach)
        picked = 0
        for batch in pairs:
            description = "，".join(
                f"格子 {slot.index} 用{('左手' if plan.arm == 'left' else '右手')} "
                f"肩距 {plan.distance:.2f} m"
                for slot, plan in batch
            )
            self.get_logger().info(f"同一趟先取后放：{description}")
            for offset, (slot, plan) in enumerate(batch):
                if not self._pick_from_stand(plan.arm):
                    self.get_logger().error(
                        f"第 {slot.index} 格取料失败，停止。取料点不再生成新箱子。"
                    )
                    return
                picked += 1
                if offset == 0 and len(batch) > 1:
                    if not self._spawn_box(picked):
                        return
            for slot, plan in batch:
                if not self._place_held(slot, plan):
                    self.get_logger().error(
                        f"第 {slot.index} 格失败，停止。取料点不再生成新箱子。"
                    )
                    return
            if picked < len(slots) and not self._spawn_box(picked):
                return
        self.get_logger().info("可达格子已经堆完。")

    def _spawn_box(self, index):
        message = Int32()
        message.data = int(index)
        self.spawn.publish(message)
        if not self.wait_ready(index, 8.0):
            self.get_logger().error(f"新箱子 {index} 没有出现。")
            return False
        self.world_boxes.append(("pickup", pickup_center(), BOX))
        if not self._sync_scene():
            return False
        self.sleep(0.4)
        return True

    def _pick_from_stand(self, arm_name):
        plan = pick_plan(arm_name)
        self._use(arm_name)
        self.waist_yaw = float(plan.waist)
        base = plan.base
        if (
            arm_name == "left"
            and self.home_pose is not None
            and math.hypot(self.home_pose[0] - base[0], self.home_pose[1] - base[1]) < 0.25
        ):
            base = self.home_pose
        self.get_logger().info(
            f"{'左手' if arm_name == 'left' else '右手'}去取料，"
            f"底盘 ({base[0]:.3f}, {base[1]:.3f}) 腰 {self.waist_yaw:.2f}"
        )
        if not self._drive_base(*base):
            return False
        self._sync_scene()
        px, py = self._into_base(*pickup_center()[:2])
        box_top = pickup_box_top()
        pick_z = tool_z_for_pick()
        self._apply_measured_boxes()
        self.pick_prim = None
        self.pick_rest_z = None
        for _name, center, _size in self.world_boxes:
            if _name != "pickup":
                continue
            # The box still on the stand is whichever prim is nearest the station.
            nearest = None
            nearest_index = None
            nearest_distance = 0.14
            station = pickup_center()
            for index, pose in self.box_poses.items():
                distance = math.hypot(pose[0] - station[0], pose[1] - station[1])
                if distance < nearest_distance and abs(pose[2] - station[2]) < 0.08:
                    nearest = pose
                    nearest_index = index
                    nearest_distance = distance
            if nearest is not None:
                px, py = self._into_base(nearest[0], nearest[1])
                pick_z = nearest[2] + carried_offset()
                box_top = nearest[2] + BOX[2] / 2.0
                self.pick_prim = nearest_index
                self.pick_rest_z = nearest[2]
                self.get_logger().info(
                    f"取料对准实测箱心 ({nearest[0]:.3f}, {nearest[1]:.3f}, {nearest[2]:.3f})"
                )
            break
        self.get_logger().info(
            f"箱顶 {box_top:.3f}，密封面贴住后工具高度 {pick_z:.3f}"
        )
        # Seed the approach from an elbow that can actually sit on the box.
        # The other elbow reaches the hover and then cannot descend.
        self.pick_clearance = True
        self._publish_collision_matrix()
        goal = self._solve_arm(px, py, pick_z)
        self.pick_clearance = False
        self._publish_collision_matrix()
        if goal is not None:
            self._arm().seed = {name: goal[name] for name in self._arm().limits}
            self.get_logger().info("取料先选定能贴住纸箱的手臂姿态")
        # The other hand may already be carrying a box, so the first descent
        # sometimes stops a couple of centimetres high. One more approach from
        # the hover fixes that without abandoning the pair.
        sealed = False
        for attempt in range(2):
            hover = pick_z + APPROACH_HEIGHT
            target_z = pick_z
            if attempt:
                # The cup is already on the box. Another arm plan from this
                # stretched pose has no nearby solution and used to abort a
                # real grip. Press a little harder on the same vertical line.
                self.get_logger().warn("纸箱没离架，再压低 6 mm")
                self.set_suction(False)
                target_z = pick_z - 0.006
            elif not self._move_tool(px, py, hover, "取料上方"):
                continue
            if not self._move_tool(px, py, target_z, "取料"):
                continue
            if not self._grasp(px, py, target_z):
                continue
            if self._box_left_stand(px, py, target_z):
                sealed = True
                break
            self.get_logger().warn("吸盘报了吸住，纸箱却没离架，松开再压一次")
            self.set_suction(False)
        if not sealed:
            return False
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
            if not self._tuck_carried(carry_z):
                return False
        finally:
            self.pick_clearance = False
            self._publish_collision_matrix()
        return True

    def _box_left_stand(self, x, y, z):
        """The grip topic can stay true while the box never leaves the stand.

        A 6 cm lift of the prismatic joint is enough to see the prim rise.
        Replanning the arm here has no nearby solution and used to drop a
        real grip. If the lift itself cannot move, keep the grip.
        """
        del x, y, z
        lift = self._lift_now()
        target = min(LIFT_LIMIT, lift + 0.06)
        if target - lift <= 0.015:
            return True
        self.get_logger().info("确认带起只升升降 6 cm")
        if not self._follow_lift(target, 0.16):
            return True
        if self.pick_prim is None or self.pick_rest_z is None:
            return True
        self.sleep(0.2)
        for _ in range(6):
            rclpy.spin_once(self, timeout_sec=0.05)
        pose = self.box_poses.get(self.pick_prim)
        if pose is None:
            return True
        if pose[2] >= self.pick_rest_z + 0.03:
            self.get_logger().info(f"纸箱已离架，箱心高度 {pose[2]:.3f}")
            return True
        self.get_logger().warn(
            f"箱心还在 {pose[2]:.3f}，离架前是 {self.pick_rest_z:.3f}"
        )
        return False

    def _tuck_carried(self, carry_z):
        """Hold the box beside this shoulder so the other hand can use the stand."""
        sign = 1.0 if self.side == "left" else -1.0
        tucked = None
        for target in ((0.16, sign * 0.32), (0.24, sign * 0.46), (0.12, sign * 0.20)):
            if self._move_carry_xy(target[0], target[1], carry_z, "收臂"):
                tucked = target
                break
        if tucked is None:
            self.get_logger().error("箱子抬起来了，但收不到肩膀旁边")
            return False
        self._move_tool(tucked[0], tucked[1], max(carry_z, 0.62), "行车高度", carry_tolerance=0.06)
        return True

    def _place_held(self, slot, plan):
        self._use(plan.arm)
        self.waist_yaw = float(plan.waist)
        self.place_path = []
        park = plan.base
        place_z = tool_z_for_place(slot)
        self.get_logger().info(
            f"{'左手' if plan.arm == 'left' else '右手'}开到底盘 "
            f"({park[0]:.3f}, {park[1]:.3f}) 腰 {self.waist_yaw:.2f}，放格子 {slot.index}"
        )
        if not self._drive_base(*park):
            return False
        self._sync_scene()
        world_x, world_y = slot.x, slot.y
        support = self._measured_support(slot)
        if support is not None:
            world_x, world_y = support[0], support[1]
            place_z = support[2] + BOX[2] + carried_offset()
            self.get_logger().info(
                f"按下层实测位姿堆叠 ({support[0]:.3f}, {support[1]:.3f}, {support[2]:.3f})"
            )
        sx, sy = self._into_base(world_x, world_y)
        self.place_supports = {"stand", "pallet"}
        for name, center, _size in self.world_boxes:
            if not name.startswith("placed_"):
                continue
            if math.hypot(center[0] - slot.x, center[1] - slot.y) < 0.04 and center[2] < slot.z - 0.04:
                self.place_supports.add(name)
        self._publish_collision_matrix()
        self.get_logger().info(
            f"格子 {slot.index} 支撑 {sorted(self.place_supports)}，"
            f"放置目标 ({sx:.3f}, {sy:.3f}, {place_z:.3f})"
        )
        if not self._place_with_torso(sx, sy, place_z):
            self.get_logger().warn("这一放下没到位，底盘再靠到规划停车点后重放一次")
            if not self._drive_base(*park):
                return False
            self._sync_scene()
            sx, sy = self._into_base(world_x, world_y)
            if not self._place_with_torso(sx, sy, place_z):
                if abs(self.waist_yaw) < 0.2:
                    return False
                self.get_logger().warn("这个腰角放不下去，腰转回 0 再放一次")
                self.waist_yaw = 0.0
                if not self._place_with_torso(sx, sy, place_z):
                    return False
        if not self._settle_over(sx, sy, place_z, "放置", 0.02):
            self.get_logger().error("吸盘没有对准格子，箱子还悬在旁边，不释放。")
            return False
        released = self._release_when_settled(sx, sy, place_z)
        if released is None:
            return False
        settled = (released[0], released[1], slot.support_z + BOX[2] / 2.0)
        if not self._detach_at(slot.index, settled):
            self.get_logger().error("释放后的规划场景没有确认成功，停止撤离。")
            return False
        if not self._retreat(sx, sy, place_z + APPROACH_HEIGHT):
            self.get_logger().error("撤离未通过路径或实测位置校验，停止；具体位置见前面的日志。")
            return False
        self.get_logger().info(f"已放下格子 {slot.index}")
        return True

    def _record_place_waypoint(self):
        arm = self._current_arm()
        pose = self._measured_tool()
        if arm is None or pose is None:
            return
        position = (pose.position.x, pose.position.y, pose.position.z)
        self.place_path.append((arm, self._lift_now(), position))


    def _retreat(self, x, y, z):
        # The place descent moved only the lift, so the way out is the same
        # lift coming back up. The arm joints stay in the pose that was over
        # the slot, which keeps the cup from swinging into the stack.
        pose = self._measured_tool()
        if pose is not None and z - pose.position.z > 0.02:
            self.get_logger().info("撤离只升起升降，手臂保持放下时的姿态")
            target = min(LIFT_LIMIT, max(0.0, self._lift_now() + (z - pose.position.z)))
            if not self._follow_lift(target, 0.22):
                return False
            pose = self._measured_tool()
            if pose is None:
                self.get_logger().error("撤离缺少末端反馈，不能确认净空")
                return False
            horizontal = math.hypot(pose.position.x - x, pose.position.y - y)
            if pose.position.z >= z - 0.03 and horizontal <= 0.04:
                self._log_tool("撤离完成", x, y, z)
                return True
            self._log_tool("升降撤离不够，改由手臂抬离", x, y, z)
            return self._move_tool(x, y, z, "撤离")
        # Reuse the measured descent branch instead of asking KDL to rediscover
        # a nearby solution at the deeply folded placement pose. Each segment
        # is still planned against the scene, including the newly placed box.
        if len(self.place_path) >= 2:
            self.get_logger().info("沿放置时记录的关节姿态逐段撤离")
            for arm, lift, position in reversed(self.place_path[:-1]):
                if abs(self._lift_now() - lift) > 0.03:
                    self.get_logger().error("下降过程中升降发生变化，不能仅靠手臂反向撤离")
                    return False
                if not self._arm_at(arm, 0.02):
                    if not self._move_named_joints(arm):
                        return False
                if not self._wait_tool(*position, timeout=4.0, tolerance=0.035):
                    # Correct once toward the same recorded configuration. Do
                    # not accept controller success without measured feedback.
                    self._log_tool("撤离未到记录点，修正一次", *position)
                    if not self._move_named_joints(arm):
                        return False
                    if not self._wait_tool(*position, timeout=4.0, tolerance=0.035):
                        self._log_tool("撤离未到记录点", *position)
                        return False
                self._arm().seed = dict(arm)
            pose = self._measured_tool()
            if pose is None:
                self.get_logger().error("撤离缺少末端反馈，不能确认净空")
                return False
            # The hover was accepted with a 3.5 cm tracking tolerance on the
            # way in. Allow the same return error, but keep an independent
            # minimum-height and lateral bound against the intended hover.
            horizontal = math.hypot(pose.position.x - x, pose.position.y - y)
            if pose.position.z < z - 0.035 or horizontal > 0.06:
                self._log_tool("撤离净空不足或横向偏移过大", x, y, z)
                return False
            self._log_tool("撤离完成", x, y, z)
            return True
        # Never bypass pallet collisions or swing straight to a distant carry
        # posture when a local retreat fails.
        return self._move_tool(x, y, z, "撤离")

    def _move_named_joints(self, joints):
        constraints = Constraints()
        for name, value in joints.items():
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return self._move(self._arm().group, constraints)

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

    def _from_base(self, x, y):
        base_x, base_y, yaw = self.base_pose
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return base_x + cosine * x - sine * y, base_y + sine * x + cosine * y

    def _quat_into_base(self, quat):
        """Box orientation from the stage frame into the chassis-fixed world frame."""
        yaw = 0.0 if self.base_pose is None else self.base_pose[2]
        half = -0.5 * yaw
        cz, sz = math.cos(half), math.sin(half)
        qx, qy, qz, qw = quat
        return (
            cz * qx + sz * qy,
            cz * qy - sz * qx,
            sz * qw + cz * qz,
            cz * qw - sz * qz,
        )

    def _sync_scene(self):
        self._apply_measured_boxes()
        objects = []
        self.scene_ids = []
        for name, center, size in self.world_boxes:
            local_x, local_y = self._into_base(center[0], center[1])
            quat = (0.0, 0.0, 0.0, 1.0)
            box_size = size
            if name.startswith("placed_"):
                measured = self.box_poses.get(int(name.split("_", 1)[1]))
                if measured is not None and len(measured) >= 7:
                    quat = self._quat_into_base(measured[3:7])
                # A settled box can lean a little. The extra margin is what the
                # arm plans around, instead of the ideal slot.
                box_size = (size[0] + 0.02, size[1] + 0.02, size[2])
            objects.append(self._box(name, (local_x, local_y, center[2]), box_size, quat))
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
        """Holonomic drive. A straight line that crosses the pallet goes around it."""
        if self.base_pose is None:
            self.sleep(0.2)
        if self.base_pose is None:
            self.get_logger().error("没有底盘位姿，不能开过去")
            return False
        goal = (float(x), float(y), float(yaw))
        points = route_around(self.base_pose, goal)
        if len(points) > 1:
            self.get_logger().info(f"底盘绕开托盘，途经 {len(points) - 1} 个拐角")
        last = len(points) - 1
        for index, point in enumerate(points):
            # The last park is what puts the slot inside the measured cup
            # window. Four centimetres short turns a reachable (0.16, 0.56)
            # into a pose the wrist cannot lower onto.
            if index == last:
                ok = self._drive_straight(*point, pos_tol=0.018, yaw_tol=0.03)
            else:
                ok = self._drive_straight(*point)
            if not ok:
                return False
        return True

    def _drive_straight(self, x, y, yaw, pos_tol=0.04, yaw_tol=0.06):
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
            # Spin to the waypoint heading before rolling. Turning while
            # translating moves the steer target every cycle, the alignment
            # gate then cuts the drive, and the chassis crawls at ~0.15 m/s.
            if abs(error_yaw) > 0.12:
                forward = 0.0
                lateral = 0.0
                turn = max(-0.9, min(0.9, 1.8 * error_yaw))
            elif dist > pos_tol:
                turn = 0.0
                speed = math.hypot(forward, lateral)
                limit = 0.55 if dist > 0.25 else 0.16
                if speed > limit and speed > 1.0e-6:
                    forward *= limit / speed
                    lateral *= limit / speed
            else:
                forward = 0.0
                lateral = 0.0
                turn = 0.0 if abs(error_yaw) < yaw_tol else max(-0.6, min(0.6, 1.4 * error_yaw))
            # Keep the steer target chosen for this twist. Re-seeding it from
            # the measured angle every cycle flips a wheel between the two
            # equivalent headings, so the drive speed stays at zero.
            measured = self._measured_steer()
            angles, drives = twist_to_wheels(forward, lateral, turn, hold)
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
            ("waist_yaw_Link", self.waist_yaw),
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
            ("waist_yaw_Link", self.waist_yaw),
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

    def _follow_joints(self, client, names, targets, speed, label, finish=True):
        """Straight smoothstep in joint space. RRT between tiny goals is what wobbles."""
        if not client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(f"没有连上{label}轨迹，不能平滑运动")
            return False
        current = []
        for name in names:
            value = self._joint_position(name)
            if value is None:
                self.get_logger().error(f"{label}缺少关节 {name}")
                return False
            current.append(value)
        goal_pos = [float(targets[name]) for name in names]
        delta = max(abs(start - end) for start, end in zip(current, goal_pos))
        if delta < 0.004:
            return True
        duration = max(0.35, delta / speed)
        steps = max(6, int(duration / 0.15))
        trajectory = JointTrajectory()
        trajectory.joint_names = list(names)
        for index in range(1, steps + 1):
            fraction = index / steps
            smooth = fraction * fraction * (3.0 - 2.0 * fraction)
            point = JointTrajectoryPoint()
            point.positions = [
                start + (end - start) * smooth for start, end in zip(current, goal_pos)
            ]
            point.time_from_start.sec = int(duration * fraction)
            point.time_from_start.nanosec = int((duration * fraction % 1.0) * 1e9)
            trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self.get_logger().info(f"{label}平滑运动 {duration:.1f} 秒，幅度 {delta:.3f}")
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label}轨迹被拒绝")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None or wrapped.result.error_code != 0:
            code = None if wrapped is None else wrapped.result.error_code
            self.get_logger().error(f"{label}轨迹没有走完，错误码 {code}")
            return False
        # Goal tolerance can report success while a joint is still several
        # degrees short. One short extra move finishes that last bit.
        leftover = 0.0
        for name in names:
            value = self._joint_position(name)
            if value is not None:
                leftover = max(leftover, abs(value - targets[name]))
        if finish and leftover > 0.03:
            return self._follow_joints(client, names, targets, max(speed * 0.5, 0.02), label, False)
        return True

    def _follow_lift(self, lift, speed):
        pitch = self._joint_position("waist_pitch_Link")
        yaw = self._joint_position("waist_yaw_Link")
        targets = {
            "linear_Link": min(LIFT_LIMIT, max(0.0, float(lift))),
            "waist_pitch_Link": 0.0 if pitch is None else pitch,
            "waist_yaw_Link": 0.0 if yaw is None else yaw,
        }
        return self._follow_joints(
            self.torso_traj,
            ("linear_Link", "waist_pitch_Link", "waist_yaw_Link"),
            targets,
            speed,
            "升降",
        )

    def _follow_arm(self, joints, speed):
        return self._follow_joints(self._arm().trajectory, tuple(self._arm().limits), joints, speed, "手臂")

    def _place_with_torso(self, x, y, z):
        """Hover with one smooth arm move, then drop by the lift alone.

        Re-solving the arm every few centimetres is what made the cup wobble
        and release early. The lift axis is base Z, so a fixed arm goes
        straight down.
        """
        hover_z = z + APPROACH_HEIGHT
        solved = None
        lift_hover = None
        # Highest lift that can still reach this slot wins: the descent is then
        # only the prismatic joint. A far column cannot reach while the lift is high.
        for candidate in (0.28, 0.20, 0.12, 0.06, 0.0):
            solved = self._solve_arm(x, y, hover_z, fixed_lift=candidate)
            if solved is not None:
                lift_hover = candidate
                break
        if solved is None:
            for candidate in (0.12, 0.06, 0.0, 0.20):
                solved = self._solve_arm(x, y, z, fixed_lift=candidate)
                if solved is not None:
                    lift_hover = candidate
                    hover_z = z
                    self.get_logger().info("上方没有逆解，直接平滑到放置高度")
                    break
        if solved is None:
            self.get_logger().warn("升降上没有逆解，退回手臂分段放下")
            if not self._move_tool(x, y, z + APPROACH_HEIGHT, "放置上方"):
                return False
            return self._move_tool(x, y, z, "放置")
        self.get_logger().info(
            f"放置升到 {lift_hover:.3f} 后只降升降，目标高度 {z:.3f}"
        )
        if not self._follow_lift(lift_hover, 0.22):
            return False
        if not self._follow_arm(solved, 0.8):
            return False
        self._arm().seed = {name: solved[name] for name in self._arm().limits}
        if not self._wait_tool(x, y, hover_z, 3.0, 0.03):
            self._log_tool("放置上方", x, y, hover_z)
        pose = self._measured_tool()
        if pose is None:
            return False
        lift_fix = self._lift_now() + (hover_z - pose.position.z)
        lift_fix = min(LIFT_LIMIT, max(0.0, lift_fix))
        if abs(lift_fix - self._lift_now()) > 0.01 and not self._follow_lift(lift_fix, 0.18):
            return False
        pose = self._measured_tool()
        if pose is None:
            return False
        horizontal = math.hypot(pose.position.x - x, pose.position.y - y)
        if horizontal > 0.015:
            self.get_logger().warn(f"到位后横向还差 {horizontal * 100:.1f} cm，手臂微调一次")
            self._move_tool_direct(x, y, pose.position.z, "放置对准", 0.015, False, wait=4.0)
            pose = self._measured_tool()
            if pose is None:
                return False
            horizontal = math.hypot(pose.position.x - x, pose.position.y - y)
            if horizontal > 0.02:
                self._log_tool("放置上方偏差过大", x, y, hover_z)
                return False
        self.place_path = []
        self._record_place_waypoint()
        self.get_logger().info(f"升降分步放到 {z:.3f}，偏了就先拉回再降")
        # One long lift move lets a brush against the previous box fold the arm
        # and the cup finishes centimetres high. Short steps stop that early.
        for _ in range(16):
            pose = self._measured_tool()
            if pose is None:
                return False
            horizontal = math.hypot(pose.position.x - x, pose.position.y - y)
            high = pose.position.z - z
            self._log_tool("放置", x, y, z)
            if horizontal <= 0.018 and abs(high) <= 0.008:
                self._record_place_waypoint()
                return True
            # Under 2 cm is already inside the release gate. Nudging a
            # fully stretched arm here never closes the last centimetre
            # and blocks the lift drop.
            if horizontal > 0.018:
                aim_z = z if high < 0.025 else pose.position.z
                if not self._aim_arm(x, y, aim_z, pose.position.z):
                    break
                pose = self._measured_tool()
                if pose is None:
                    return False
                moved = math.hypot(pose.position.x - x, pose.position.y - y)
                if moved > horizontal - 0.005:
                    self.get_logger().warn("手臂补这一步没有更近，不再硬凑")
                    break
                continue
            if high <= 0.008:
                break
            step = min(0.06, high)
            target_lift = self._lift_now() - step
            if target_lift < 0.0:
                if self._lift_now() > 0.008 and not self._follow_lift(0.0, 0.15):
                    return False
                self.get_logger().info("升降已到最低，剩下的高度由手臂平滑补上")
                if not self._aim_arm(x, y, z, pose.position.z):
                    break
                continue
            before_z = pose.position.z
            if not self._follow_lift(max(0.0, target_lift), 0.15):
                return False
            pose = self._measured_tool()
            if pose is None:
                return False
            dropped = before_z - pose.position.z
            if dropped < 0.5 * step:
                self.get_logger().warn(
                    f"升降降了 {step * 100:.1f} cm，末端只降了 {dropped * 100:.1f} cm，改由手臂下降"
                )
                if not self._aim_arm(x, y, z, pose.position.z):
                    break
        pose = self._measured_tool()
        if pose is None:
            return False
        horizontal = math.hypot(pose.position.x - x, pose.position.y - y)
        high = pose.position.z - z
        if horizontal <= 0.02 and high <= 0.012:
            self._record_place_waypoint()
            return True
        self._log_tool("放置偏差过大", x, y, z)
        return False

    def _aim_arm(self, x, y, z, fallback_z):
        """Smooth arm move at the current lift. Tries the target height first."""
        lift = self._lift_now()
        solved = self._solve_arm(x, y, z, fixed_lift=lift)
        if solved is None and abs(z - fallback_z) > 0.005:
            solved = self._solve_arm(x, y, fallback_z, fixed_lift=lift)
        if solved is None:
            self.get_logger().warn("放下时手臂没有逆解")
            return False
        return self._follow_arm(solved, 0.6)

    def _current_arm(self):
        if self.joint_state is None:
            return None
        positions = dict(zip(self.joint_state.name, self.joint_state.position))
        if any(name not in positions for name in self._arm().limits):
            return None
        return {name: float(positions[name]) for name in self._arm().limits}

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
        set_joint("waist_yaw_Link", self.waist_yaw)
        state = JointState()
        state.name = names
        state.position = positions
        return state

    def _solve_arm(self, x, y, z, max_joint_delta=None, fixed_lift=None, prefer=None):
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
        if self._arm().seed is not None:
            seeds.append(self._arm().seed)
        current = self._current_arm()
        if current is not None:
            seeds.append(current)
        for _ in range(6):
            seeds.append({name: random.uniform(low, high) for name, (low, high) in self._arm().limits.items()})
        reference = prefer or current or self._arm().seed or {name: 0.0 for name in self._arm().limits}
        preferred = lift_for_tool_z(z)
        # Stay at the current lift for the vertical approach and retreat.
        # Changing it between the pick and the lift-off is what produced the
        # empty torso plan.
        if fixed_lift is not None:
            lifts = [min(LIFT_LIMIT, max(0.0, float(fixed_lift)))]
        else:
            current_lift = min(LIFT_LIMIT, max(0.0, self._lift_now()))
            # While a box is on the cup, changing the lift between two short
            # moves is what makes the next inverse solution disappear.
            lifts = [current_lift]
            if not self._arm().holding:
                for value in self._candidate_lifts(z):
                    if all(abs(value - other) > 0.03 for other in lifts):
                        lifts.append(value)
        rejected = {"ik": 0, "collision": 0, "fk": 0, "joint_delta": 0}
        for lift_index, lift in enumerate(lifts):
            arm_seeds = seeds if lift_index == 0 else seeds[:3]
            best = None
            best_cost = None
            for seed in arm_seeds:
                request = GetPositionIK.Request()
                request.ik_request.group_name = self._arm().group
                request.ik_request.ik_link_name = self._arm().tool
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
                    rejected["ik"] += 1
                    continue
                check = GetStateValidity.Request()
                check.robot_state = result.solution
                check.group_name = self._arm().group
                checked = self.validity.call_async(check)
                rclpy.spin_until_future_complete(self, checked)
                validity = checked.result()
                if validity is None or not validity.valid:
                    rejected["collision"] += 1
                    continue
                if not self._tool_reaches(result.solution, x, y, z):
                    rejected["fk"] += 1
                    continue
                positions = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
                if any(name not in positions for name in self._arm().limits):
                    continue
                solution = {name: float(positions[name]) for name in self._arm().limits}
                solution["linear_Link"] = float(lift)
                solution["waist_pitch_Link"] = 0.0
                solution["waist_yaw_Link"] = float(self.waist_yaw)
                # A short descent must stay on the current elbow. A distant
                # solution swings the cup off the box and the loose controller
                # goal then reports success anyway.
                if max_joint_delta is not None and current is not None:
                    if any(
                        abs(solution[name] - current[name]) > max_joint_delta for name in self._arm().limits
                    ):
                        rejected["joint_delta"] += 1
                        continue
                cost = sum(abs(solution[name] - reference[name]) for name in self._arm().limits)
                cost += 3.0 * abs(solution["linear_Link"] - preferred)
                if best_cost is None or cost < best_cost:
                    best = solution
                    best_cost = cost
            if best is not None:
                return best
        self.get_logger().warn(f"逆解候选全部失败 {rejected}，关节步长限制 {max_joint_delta}")
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
        request.fk_link_names = [self._arm().tool]
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
            if self._at_tool(x, y, z, tolerance):
                return True
            self.sleep(0.1)
        return False

    def _place_sample(self, x, y, z):
        pose = self._measured_tool()
        if pose is None:
            return None
        current = (pose.position.x, pose.position.y, pose.position.z)
        horizontal = math.hypot(current[0] - x, current[1] - y)
        return current, horizontal, current[2] - z

    def _wait_stopped(self, x, y, z, hold=0.25, timeout=3.0):
        """Last sample after the tool stops moving. It may still be short of the slot."""
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        since = None
        previous = None
        last = None
        while rclpy.ok() and self.get_clock().now() < deadline:
            sample = self._place_sample(x, y, z)
            if sample is None:
                since = None
                self.sleep(0.05)
                continue
            current = sample[0]
            jumped = previous is not None and math.dist(current, previous) > 0.002
            previous = current
            last = sample
            if not jumped:
                if since is None:
                    since = self.get_clock().now()
                elif self.get_clock().now() - since >= Duration(seconds=hold):
                    return sample
            else:
                since = None
            self.sleep(0.05)
        return last

    def _place_pose_ok(self, horizontal, high):
        # 2 cm is what this wrist can hold at the deck. More than that, or
        # still above the slot, drops the box on its edge.
        return horizontal <= 0.02 and high <= 0.012

    def _release_when_settled(self, x, y, z):
        """Open the cup once the lip has stopped and is on the slot.

        Returns the world XY where the box actually is, or None to keep holding.
        """
        self.get_logger().info("吸盘保持，等末端停稳再松开")
        sample = self._wait_stopped(x, y, z)
        if sample is not None and not self._place_pose_ok(sample[1], sample[2]):
            self.get_logger().warn(
                f"停住后还差 横向 {sample[1] * 100:.1f} cm、高度 {sample[2] * 100:.1f} cm，再对准一次"
            )
            self._move_tool_direct(x, y, z, "放置对准", 0.02, False, wait=4.0)
            sample = self._wait_stopped(x, y, z)
        if sample is None or not self._place_pose_ok(sample[1], sample[2]):
            self._log_tool("放置还没停稳", x, y, z)
            self.get_logger().error("末端还没停在格子上，不释放。")
            return None
        current, horizontal, high = sample
        self.get_logger().info(
            f"放置停稳 横向 {horizontal * 100:.1f} cm，高度差 {high * 100:.1f} cm，松开"
        )
        world_x, world_y = self._from_base(current[0], current[1])
        self.set_suction(False)
        # Let the box sit on the riser before the cup lifts away.
        self.sleep(0.6)
        return world_x, world_y

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

    def _hold_xy(self, x, y, z):
        """Stay over the slot before dropping further. A sideways lag drops the box."""
        for _ in range(2):
            pose = self._measured_tool()
            if pose is None:
                return False
            horizontal = math.hypot(pose.position.x - x, pose.position.y - y)
            high = pose.position.z - z
            if horizontal <= 0.012 and high <= 0.012:
                return True
            self.get_logger().warn(
                f"放置横向 {horizontal * 100:.1f} cm、偏高 {max(high, 0.0) * 100:.1f} cm，"
                "停在这一高度再对准"
            )
            self._move_tool_direct(x, y, z, "放置对准", 0.012, False, wait=4.0)
        pose = self._measured_tool()
        if pose is None:
            return False
        horizontal = math.hypot(pose.position.x - x, pose.position.y - y)
        if horizontal > 0.012 or pose.position.z > z + 0.012:
            self._log_tool("放置没有对准", x, y, z)
            return False
        return True

    def _settle_over(self, x, y, z, label, tolerance):
        """Do not open the cup until the measured lip is on this pose."""
        for _ in range(3):
            if self._at_tool(x, y, z, tolerance):
                self._log_tool(f"{label} 已对准", x, y, z)
                return True
            self.get_logger().warn(f"{label} 还没对准，再走一次")
            self._move_tool_direct(x, y, z, label, tolerance, False, wait=4.0)
        if self._at_tool(x, y, z, tolerance):
            self._log_tool(f"{label} 已对准", x, y, z)
            return True
        self._log_tool(f"{label} 偏差过大", x, y, z)
        return False

    def _pick_by_lift(self, x, y, z):
        pose = self._measured_tool()
        if pose is None:
            return False
        horizontal = math.hypot(pose.position.x - x, pose.position.y - y)
        drop = pose.position.z - z
        if horizontal > 0.03 or drop <= 0.02:
            return False
        target_lift = self._lift_now() - drop
        if target_lift < -0.01:
            return False
        self.get_logger().info(f"取料只降升降 {drop:.3f} m，手臂保持在箱子上方")
        if not self._follow_lift(max(0.0, target_lift), 0.16):
            return False
        pose = self._measured_tool()
        if pose is not None:
            fix = min(LIFT_LIMIT, max(0.0, self._lift_now() + (z - pose.position.z)))
            if abs(fix - self._lift_now()) > 0.01 and not self._follow_lift(fix, 0.16):
                return False
        if self._at_tool(x, y, z, 0.03):
            self._log_tool("取料", x, y, z)
            return True
        self.get_logger().warn("升降取料后还有偏差，手臂再对一次")
        return bool(self._aim_arm(x, y, z, z) and self._at_tool(x, y, z, 0.03))

    def _arm_path_stays_over(self, joints, x, y, floor_z):
        """True when a straight joint move keeps the cup above the box."""
        current = self._current_arm()
        if current is None:
            return False
        lift = self._lift_now()
        for index in range(1, 7):
            fraction = index / 6.0
            smooth = fraction * fraction * (3.0 - 2.0 * fraction)
            mid = {
                name: current[name] + (joints[name] - current[name]) * smooth
                for name in self._arm().limits
            }
            state = RobotState()
            state.joint_state = self._ik_seed(mid, lift)
            pose = self._forward_tool(state)
            if pose is None:
                return False
            if math.hypot(pose.position.x - x, pose.position.y - y) > 0.035:
                return False
            if pose.position.z < floor_z - 0.02:
                return False
        return True

    def _move_vertical(self, x, y, z, label):
        pose = self._measured_tool()
        cursor = pose.position.z
        direction = 1.0 if z > cursor else -1.0
        # The forearm meets the pickup box on the way down. Allow that contact
        # so inverse kinematics can keep the same elbow instead of swinging aside.
        previous_clearance = self.pick_clearance
        self.pick_clearance = label == "取料" or previous_clearance
        self._publish_collision_matrix()
        try:
            # The wrist is already over the box. Dropping the lift keeps that
            # pose; re-solving the arm here is what runs out of a nearby elbow.
            if label == "取料" and self._pick_by_lift(x, y, z):
                return True
            switched = False
            while True:
                remaining = z - cursor
                # Place creeps down. A 4 cm step finishes in the controller
                # while the box is still in the air, and the cup lets go early.
                step = 0.02 if label == "放置" else 0.04
                if abs(remaining) <= step + 0.005:
                    target_z = z
                else:
                    target_z = cursor + direction * step
                # The last centimetre has to reach the cardboard. A 2.5 cm
                # sphere used to accept the cup while it was still in the air.
                tolerance = 0.01 if abs(target_z - z) <= 1e-4 else 0.02
                if str(label).startswith("取料") or str(label).startswith("再吸取"):
                    if abs(target_z - z) <= 1e-4:
                        tolerance = 0.004
                if label == "放置":
                    tolerance = 0.012 if abs(target_z - z) <= 1e-4 else 0.018
                reached = self._move_tool_direct(x, y, target_z, label, tolerance, False)
                if not reached and label == "取料" and not switched:
                    switched = True
                    free = self._solve_arm(x, y, target_z, fixed_lift=self._lift_now())
                    if (
                        free is not None
                        and self._arm_path_stays_over(free, x, y, target_z)
                        and self._follow_arm(free, 0.2)
                    ):
                        self.get_logger().info("取料换了手臂姿态，吸盘仍在纸箱上方")
                        self._arm().seed = {name: free[name] for name in self._arm().limits}
                        reached = True
                    else:
                        self.get_logger().warn("换姿态会让吸盘离开纸箱，停止")
                if label == "放置":
                    if not self._hold_xy(x, y, target_z):
                        return False
                elif not reached:
                    return False
                if label == "放置":
                    self._record_place_waypoint()
                cursor = target_z
                if abs(cursor - z) <= 1e-4:
                    return True
        finally:
            self.pick_clearance = previous_clearance
            self._publish_collision_matrix()

    def _move_tool_direct(
        self, x, y, z, label, tolerance=0.035, allow_pose_fallback=True, wait=2.0
    ):
        # A short clearance move often has no nearby inverse solution even
        # though the cup is already inside the tolerance. Forcing one swings.
        # Picking is the exception: 1 cm short leaves the seal in the air.
        accept = tolerance
        if str(label).startswith("取料") or str(label).startswith("再吸取"):
            accept = min(float(tolerance), 0.01)
        if self._at_tool(x, y, z, accept):
            self._log_tool(label, x, y, z)
            return True
        measured = self._measured_tool()
        max_joint_delta = None
        gap = 1.0
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
            arm_only = {name: solved[name] for name in self._arm().limits}
            if not self._move_torso(solved["linear_Link"]):
                return False
            # The second box is a short drop. Full speed ends the trajectory
            # before Isaac's arm arrives, and the cup opens on the way down.
            velocity = 0.25 if str(label).startswith("放置") else 0.45
            if (str(label).startswith("取料") or str(label).startswith("再吸取")) and gap < 0.03:
                velocity = 0.16
            if str(label).startswith("放置"):
                wait = max(wait, 2.0)
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
            if self._move(self._arm().group, constraints, velocity) or self._at_tool(x, y, z, tolerance):
                if self._wait_tool(x, y, z, wait, tolerance):
                    self._arm().seed = dict(arm_only)
                    self._log_tool(label, x, y, z)
                    return True
                # Carry may lag a little. Placement must not: a 4 cm miss
                # drops the box beside the previous one.
                if (
                    self._arm().holding
                    and not label.startswith("放置")
                    and self._at_tool(x, y, z, tolerance + 0.02)
                ):
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
        position.link_name = self._arm().tool
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
        orientation.link_name = self._arm().tool
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
        velocity = 0.25 if str(label).startswith("放置") else 0.45
        return self._move(self._arm().group, constraints, velocity)

    def _arm_at(self, joints, tolerance):
        for name, value in joints.items():
            current = self._joint_position(name)
            if current is None or abs(current - value) > tolerance:
                return False
        return True

    def _move(self, group, constraints, velocity=0.15):
        for attempt in range(3):
            code = self._move_once(group, constraints, velocity)
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

    def _move_once(self, group, constraints, velocity=0.15):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = group
        request.num_planning_attempts = 10
        request.allowed_planning_time = 8.0
        request.max_velocity_scaling_factor = velocity
        request.max_acceleration_scaling_factor = velocity
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

    def _box(self, object_id, center, size, quat=(0.0, 0.0, 0.0, 1.0)):
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
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
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
        links = []
        open_links = []
        place_links = []
        for arm in self._arms.values():
            arm_links = list(arm.ee)
            if self.pick_clearance and arm.name == self.side:
                arm_links = arm_links + arm.upper
            # The Robotiq meshes are inactive in the suction stage, but MoveIt
            # still carries them on the wrist. They have to be allowed against
            # the box or the descent stops about 10 cm up.
            grippers = [
                name for name in GRIPPER_LINKS
                if name.startswith("left_" if arm.name == "left" else "right_")
            ]
            links.extend(arm_links)
            links.extend(grippers)
            if arm.holding and arm.name == self.side:
                place_links.extend(arm_links + grippers)
            elif not arm.holding:
                open_links.extend(arm_links + grippers)
        names = list(self._srdf_names)
        extra = [arm.carried for arm in self._arms.values() if arm.holding]
        for name in links + list(self.scene_ids) + extra:
            if name not in names:
                names.append(name)
        active_id = self._arm().carried if self._arm().holding else None
        hands = {
            arm.carried: set(arm.touch + GRIPPER_LINKS)
            for arm in self._arms.values()
            if arm.holding
        }
        matrix = AllowedCollisionMatrix()
        matrix.entry_names = names
        for name in names:
            entry = AllowedCollisionEntry()
            enabled = []
            for other in names:
                adjacent = (name, other) in self._srdf_allowed
                # The empty cup may touch the scene. The cup that is placing
                # may touch only the stand, the pallet, and the box under it.
                # The other hand's box stays solid against the stack.
                cup_on_object = (
                    (name in open_links and other in self.scene_ids)
                    or (other in open_links and name in self.scene_ids)
                    or (name in place_links and other in self.place_supports)
                    or (other in place_links and name in self.place_supports)
                )
                held_on_support = active_id is not None and (
                    (name == active_id and other in self.place_supports)
                    or (other == active_id and name in self.place_supports)
                )
                held_on_hand = False
                for carried_id, hand in hands.items():
                    if (name == carried_id and other in hand) or (other == carried_id and name in hand):
                        held_on_hand = True
                        break
                enabled.append(adjacent or cup_on_object or held_on_support or held_on_hand)
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
        arm = self._arm()
        carried = CollisionObject()
        carried.header.frame_id = arm.tool
        carried.id = arm.carried
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        # 2 cm inset on each side. Packed slots share a face, so a full-size
        # carried box touches its neighbors even when it is centered. The inset
        # still blocks a descent that has drifted more than 2 cm into one.
        primitive.dimensions = [BOX[0] - 0.04, BOX[1] - 0.04, BOX[2]]
        pose = Pose()
        # Box center is along tool +Z. Rx(180) keeps the box upright in the world.
        pose.position.z = carried_offset()
        pose.orientation.x = 1.0
        pose.orientation.w = 0.0
        carried.primitives = [primitive]
        carried.primitive_poses = [pose]
        carried.operation = CollisionObject.ADD
        attached = AttachedCollisionObject()
        attached.link_name = arm.tool
        attached.object = carried
        attached.touch_links = arm.touch + GRIPPER_LINKS
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [remove]
        scene.robot_state.attached_collision_objects = [attached]
        scene.robot_state.is_diff = True
        if not self._call_scene(scene):
            return
        self._arm().holding = True
        self.place_supports = {"stand", "pallet"}
        self.world_boxes = [item for item in self.world_boxes if item[0] != "pickup"]
        if "pickup" in self.scene_ids:
            self.scene_ids.remove("pickup")
        self._publish_collision_matrix()

    def _detach_at(self, index, center):
        arm = self._arm()
        remove = CollisionObject()
        remove.id = arm.carried
        remove.operation = CollisionObject.REMOVE
        attached = AttachedCollisionObject()
        attached.link_name = arm.tool
        attached.object = remove
        placed_id = f"placed_{index}"
        local_x, local_y = self._into_base(center[0], center[1])
        placed = self._box(placed_id, (local_x, local_y, center[2]), BOX)
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [placed]
        scene.robot_state.attached_collision_objects = [attached]
        scene.robot_state.is_diff = True
        if not self._call_scene(scene):
            return False
        # Detaching with REMOVE transfers the attached shape back to the world
        # under its old ID. Remove that duplicate in a SECOND scene update:
        # world updates are processed before attached-object updates.
        cleanup = PlanningScene()
        cleanup.is_diff = True
        cleanup.world.collision_objects = [remove]
        if not self._call_scene(cleanup):
            return False
        if not self.get_scene.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("无法核验释放场景：get_planning_scene 不可用")
            return False
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        future = self.get_scene.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            return False
        world = {obj.id for obj in result.scene.world.collision_objects}
        attached_ids = {
            obj.object.id for obj in result.scene.robot_state.attached_collision_objects
        }
        if arm.carried in world or arm.carried in attached_ids or placed_id not in world:
            self.get_logger().error(
                f"释放场景异常：world={sorted(world)}，attached={sorted(attached_ids)}"
            )
            return False
        self._arm().holding = False
        self.world_boxes.append((placed_id, center, BOX))
        self.scene_ids.append(placed_id)
        if not self._publish_collision_matrix():
            return False
        self.get_logger().info(f"释放场景已确认：{placed_id} 已落地，{arm.carried} 已清除")
        return True


def main():
    rclpy.init()
    node = StackBoxes()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
