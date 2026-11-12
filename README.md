# wheel_robot_moveit

ROS 2 Humble workspace for the dual-arm wheeled robot. It contains the URDF, meshes, MoveIt 2 configuration, Nav2 bringup, and Isaac Sim USD stages. Isaac Lab training code is not included.

The robot has a prismatic lift, a two-axis waist, two 7-DOF arms, and a Robotiq 2F-85 on each wrist. Four swerve modules drive the base. MoveIt plans the torso, arms, and grippers. Isaac Sim executes the motion over ROS 2 topics. The wheels are commanded separately through the swerve controller.

This workspace was copied from `moveit2_UR5` so ROS and Isaac Sim can be used without the Isaac Lab packages.

## Layout

| Path | Contents |
| --- | --- |
| `src/robot_description` | URDF/xacro and meshes for the wheel robot and both Robotiq 2F-85 grippers |
| `src/wheel_robot_moveit_config` | MoveIt 2 configuration, ros2_control, RViz, and launch files |
| `src/wheel_robot_nav` | Wheel odometry, 16-beam localization, swerve controller, and Nav2 |
| `wheel_robot_moveit.usd` | Isaac Sim stage: robot, physics, arm Action Graph, lidar, and wheel topics |
| `wheel_robot_suction.usd` | Copy of the same robot. Both Robotiq grippers are off and each wrist has a suction cup |
| `wheel_robot_suction_stack.usd` | Suction scene plus a pallet, a fixed pickup stand, and one box. The next box appears at that stand only after a successful place |
| `wheel_robot_nav_world.usd` | Navigation stage: the same robot, plus an IMU, walls, and obstacles |
| `assets/grippers/robotiq_85_gripper.usd` | Standalone Robotiq 2F-85 that can be mounted on other robots |
| `assets/grippers/suction_cup.usd` | Standalone suction-cup stage. The origin is the mount flange and the seal faces +Z. An Action Graph opens and closes it |

Not copied: `isaaclab/`, gear-assembly policies, UR5, or `mbot_with_wheels`.

## Requirements

- Ubuntu with ROS 2 Humble and MoveIt 2
- `colcon`, `xacro`
- Navigation also needs `nav2_bringup`, `nav2_smac_planner`, and `teleop_twist_keyboard`
- Isaac Sim 4.5 to execute motion in simulation (default `/home/cym/isaac-sim-4.5.0`; override with `ISAAC_SIM`)

Install the usual MoveIt / Nav2 packages on Humble:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-topic-based-ros2-control \
  ros-humble-nav2-bringup \
  ros-humble-nav2-smac-planner \
  ros-humble-teleop-twist-keyboard \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro
```

Build with the system Python, not a conda or Isaac Sim interpreter.

## Build

```bash
source /opt/ros/humble/setup.bash
cd /home/cym/isaac_sim_projects/wheel_robot_moveit
colcon build --symlink-install
source install/setup.bash
```

Source both files in every new terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/cym/isaac_sim_projects/wheel_robot_moveit/install/setup.bash
```

## Inspect the model (ROS only)

This does not need Isaac Sim. It publishes the URDF, a joint-state GUI, and RViz.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch wheel_robot_moveit_config rsp.launch.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
ros2 run rviz2 rviz2
```

In RViz set **Fixed Frame** to `world`, then add **RobotModel** and **TF**. Drag the sliders in the joint GUI to move the lift, waist, arms, and gripper knuckles.

To print the expanded URDF:

```bash
xacro "$(ros2 pkg prefix robot_description)/share/robot_description/urdf/wheel_robot/wheel_robot_with_grippers.xacro"
```

The source files are:

- body: `src/robot_description/urdf/wheel_robot/wheel_robot_body.urdf.xacro`
- grippers: `src/robot_description/urdf/wheel_robot/wheel_robot_grippers.urdf.xacro`
- top-level xacro: `src/robot_description/urdf/wheel_robot/wheel_robot_with_grippers.xacro`
- meshes: `src/robot_description/meshes/wheel_robot/` and `src/robot_description/meshes/ur5/`

## MoveIt in RViz

`demo.launch.py` starts `robot_state_publisher`, `move_group`, RViz, and the MoveIt warehouse. ros2_control uses `topic_based_ros2_control/TopicBasedSystem`, which talks to `/isaac_joint_states` and `/isaac_joint_commands`. You can still open the MotionPlanning panel and plan; trajectory execution waits for those Isaac topics.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheel_robot_moveit_config demo.launch.py
```

In the MotionPlanning panel:

1. Planning Group: `left_arm`, `right_arm`, or `torso`.
2. Goal State: `left_home` / `right_home`, or `nav_tuck`.
3. Plan, then Execute if a robot is publishing joint states.

RViz **Fixed Frame** is `world`. Do not start `demo.launch.py` together with `isaac_moveit.launch.py` or either navigation launch; each starts its own `controller_manager` or `robot_state_publisher`.

Useful pieces if you want to start them separately:

```bash
ros2 launch wheel_robot_moveit_config rsp.launch.py
ros2 launch wheel_robot_moveit_config move_group.launch.py
ros2 launch wheel_robot_moveit_config moveit_rviz.launch.py
```

## Isaac Sim stages

The USD files are self-contained crates. Open them from this workspace; you do not need `moveit2_UR5`.

| File | Use |
| --- | --- |
| `wheel_robot_moveit.usd` | Arms, torso, and grippers with MoveIt. Isaac publishes `/isaac_joint_states` and subscribes to `/isaac_joint_commands`. |
| `wheel_robot_suction.usd` | Copy with suction cups instead of the grippers. Arm joints match the gripper stage. After Play, `left_suction_cmd` / `right_suction_cmd` (`std_msgs/Bool`, `true` grips) open and close the cups. |
| `wheel_robot_suction_stack.usd` | Suction scene plus the 1200×1000 mm pallet and a fixed pickup stand. Use `ros2 run wheel_robot_stack stack_boxes`. |
| `wheel_robot_nav_world.usd` | Same robot plus IMU, 16-beam lidar, walls, and obstacles. Use this for Nav2 and for MoveIt + navigation in one window. |
| `assets/grippers/robotiq_85_gripper.usd` | One Robotiq 2F-85. The origin is the tool flange. Drive only `robotiq_85_left_knuckle_joint` (0° to 46°). |
| `assets/grippers/suction_cup.usd` | Standalone suction cup. The origin is the mount flange and the seal faces +Z. After Play, `suction_cmd` (`std_msgs/Bool`) or the `suctionClosed` attribute opens and closes it. |

Default Isaac Sim install:

```bash
ISAAC_SIM=${ISAAC_SIM:-/home/cym/isaac-sim-4.5.0}
```

Open a stage from the GUI (**File → Open**), or from the command line:

```bash
cd /home/cym/isaac_sim_projects/wheel_robot_moveit
ISAAC_SIM=${ISAAC_SIM:-/home/cym/isaac-sim-4.5.0}
"$ISAAC_SIM/isaac-sim.sh" "$(pwd)/wheel_robot_moveit.usd"
```

Press **Play** after the stage loads. The ROS 2 bridge in the stage starts publishing and subscribing. Keep Isaac Sim playing for the whole ROS session.

## Run with Isaac Sim

1. Open `wheel_robot_moveit.usd` in Isaac Sim and press Play.
2. Isaac publishes `/isaac_joint_states` and subscribes to `/isaac_joint_commands`.
3. In another terminal from this workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheel_robot_moveit_config isaac_moveit.launch.py
```

Stop any previous launch first so only one `controller_manager` is running.

### Controllers

| Controller | Type | Joints |
| --- | --- | --- |
| `left_arm_controller` | `FollowJointTrajectory` | `Link1_L_joint` … `Link7_L_joint` |
| `right_arm_controller` | `FollowJointTrajectory` | `Link1_R_joint` … `Link7_R_joint` |
| `torso_controller` | `FollowJointTrajectory` | `linear_Link`, `waist_pitch_Link`, `waist_yaw_Link` |
| `left_gripper_controller` | `GripperActionController` | `left_robotiq_85_left_knuckle_joint` |
| `right_gripper_controller` | `GripperActionController` | `right_robotiq_85_left_knuckle_joint` |
| `joint_state_broadcaster` | joint states | all reported interfaces |

The eight wheel joints are not in MoveIt or ros2_control. The swerve controller drives them. Each gripper plans only the driven knuckle; the other finger follows through a mimic joint. Open is `0`, closed is `0.7` rad.

### Planning groups

| Group | Links / joints |
| --- | --- |
| `torso` | `linear_Link`, `waist_pitch_Link`, `waist_yaw_Link` |
| `left_arm` | chain `waist_yaw_Link` → `tool0_L` |
| `right_arm` | chain `waist_yaw_Link` → `tool0_R` |
| `left_gripper` | `left_robotiq_85_left_knuckle_joint` |
| `right_gripper` | `right_robotiq_85_left_knuckle_joint` |
| `left_arm_with_torso` | torso + left arm |
| `right_arm_with_torso` | torso + right arm |

Kinematics is KDL. Named arm states include `left_home`, `right_home`, and `nav_tuck` (arms folded beside the chassis for driving).

## Navigation

Open `wheel_robot_nav_world.usd` in this workspace in Isaac Sim and press Play, then:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheel_robot_nav navigation.launch.py
```

Localization is lidar-inertial scan matching (`lidar_localization`). Wheel speed supplies `vx` and `vy`, and the IMU gyro supplies yaw rate, which predicts the next pose. The 16-beam cloud then aligns to a voxel map with point-to-plane matching and corrects `x`, `y`, and yaw. Returns between 0.25 m and 1.7 m above the ground also paint `/map`. `map` → `odom` is identity; the matcher publishes `odom` → `base_link`. Nav2 plans with Smac 2D A* and tracks with omnidirectional MPPI. Both costmaps subscribe to `/points`.

Do not run this together with `isaac_moveit.launch.py` or `demo.launch.py`. Navigation publishes `odom` → `base_link`; those MoveIt launches use a fixed `world` link.

Keyboard teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Turn through a short arc so the walls enter `/map`, then send a Nav2 Goal inside that area. Keep the lift at its lower stop while navigating so the lidar stays above the arms.

### Topics

| Topic | Direction | Role |
| --- | --- | --- |
| `/isaac_joint_states` | Isaac → ROS | All joints, for odometry and `robot_state_publisher` |
| `/isaac_joint_commands` | ROS → Isaac | Arm, torso, and gripper position commands |
| `/isaac_wheel_steer` | ROS → Isaac | Position commands for the four steer joints |
| `/isaac_wheel_drive` | ROS → Isaac | Velocity commands for the four drive joints |
| `/odom` | wheel odometry | Integrated wheel speed; does not publish TF |
| `/imu` | Isaac | IMU on `imu_link`, aligned with `base_link` |
| `/odometry/filtered` | 16-beam localization | Predicted and matched pose; publishes `odom` → `base_link` |
| `/map` | 16-beam localization | Occupancy grid painted from the multi-beam returns |
| `/points` | Isaac | 16-beam cloud in `lidar_link` |
| `/ground_truth_odom` | Isaac | Simulated ground truth; does not publish TF |
| `/cmd_vel` | Nav2 → swerve controller | Chassis velocity `vx`, `vy`, `wz` |

## Navigation and MoveIt in one window

At the zero pose each arm sticks out about 1 m, so gaps the chassis can clear still hit a forearm. The combined launch tucks both arms with MoveIt and widens the Nav2 footprint. MoveIt plans in `odom`, using the same `odom` → `base_link` transform as localization. RViz **Fixed Frame** stays `map`.

Isaac Sim should be playing `wheel_robot_nav_world.usd`, not `wheel_robot_moveit.usd`. Stop `navigation.launch.py` or `isaac_moveit.launch.py` first. Start this only while the base is in open space; the arms swing in from the outstretched pose:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheel_robot_nav navigation_moveit.launch.py
```

Wait until the terminal prints `both arms are tucked` before sending a Nav2 goal. In RViz, **Nav2 Goal** drives the base and **Interact** drags an end effector. In the MotionPlanning panel choose `left_arm` or `right_arm`, set the goal state to `nav_tuck`, then Plan & Execute.

If the robot is already between obstacles, launch with the tuck disabled, drive clear, then tuck:

```bash
ros2 launch wheel_robot_nav navigation_moveit.launch.py tuck_arms:=false
ros2 run wheel_robot_nav tuck_arms --ros-args -p use_sim_time:=true
```

## Nodes in `wheel_robot_nav`

| Executable | Role |
| --- | --- |
| `wheel_odometry` | Integrates wheel speeds into `/odom` (no TF) |
| `lidar_localization` | Predicts with wheels + IMU, matches `/points`, publishes `odom` → `base_link` and `/map` |
| `swerve_controller` | Converts `/cmd_vel` into `/isaac_wheel_steer` and `/isaac_wheel_drive` |
| `tuck_arms` | Closes the grippers, then plans and executes `nav_tuck` on both arms |
| `odom_fusion` | Extra EKF helper; not started by the default launches |
| `imu_covariance` | Extra IMU helper; not started by the default launches |

## Suction stacking

Open `wheel_robot_suction_stack.usd`, press Play, and leave it playing. In other terminals start MoveIt, then the stack node:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheel_robot_moveit_config isaac_moveit.launch.py
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run wheel_robot_stack stack_boxes
```

Slots come from Jylänki's 2010 Maximal Rectangles / Bottom-Left rule. A 351×287 mm face tiles 3×3 on the 1200×1000 mm pallet. Boxes are 105 mm high, so 1080 mm holds 10 layers and 90 slots. The pallet is `assets/grippers/pallet.usd`, scaled to 1200×1000×125 mm and sitting on the ground. The left arm reaches the near row only: 3 boxes per layer, 30 boxes. Pickup stays at `(0.55, 0.18)` on a 0.35 m stand. Planning moves the lift together with the left arm, and holds waist pitch and yaw at 0. The next box is spawned there only after the current one is released. The left cup listens on `left_suction_cmd`. Print the slots without moving the robot: `ros2 run wheel_robot_stack stack_boxes --ros-args -p execute:=false`.

## Notes

- Use Humble’s system Python for `colcon build`. A conda or Isaac Sim interpreter will fail to find `ament` / `rclpy`.
- Restart launches after changing URDF or SRDF, or rebuild with `--symlink-install` and source `install/setup.bash` again.
- The standalone suction cup is `assets/grippers/suction_cup.usd`. It is not mounted on this robot. After Play, publish `std_msgs/Bool` on `suction_cmd` (`true` grips) or set the prim attribute `suctionClosed`. The object needs a rigid body and collision, and the sealing face has to be within about 2 cm.
- Gear-assembly training, `gear_demo`, and `wheel_robot_gear_demo.usd` live in `moveit2_UR5`, not in this workspace.
