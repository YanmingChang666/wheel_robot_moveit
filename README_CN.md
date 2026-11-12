# wheel_robot_moveit

轮式双臂机器人的 ROS 2 Humble 工作空间。这里有 URDF、网格、MoveIt 2 配置、Nav2，以及 Isaac Sim 用的 USD 场景。不含 Isaac Lab 训练代码。

机器人有升降躯干、两轴腰部、两条 7 自由度手臂，双手各一只 Robotiq 2F-85。底盘是四个舵轮。MoveIt 规划躯干、双臂和夹爪，Isaac Sim 通过 ROS 2 话题执行。轮子由舵轮控制器单独驱动，不进 MoveIt。

本目录是从 `moveit2_UR5` 里抽出的 ROS 和仿真场景，方便不带 Isaac Lab 使用。

## 目录

| 路径 | 内容 |
| --- | --- |
| `src/robot_description` | 轮式机器人和左右 Robotiq 2F-85 的 URDF/xacro 与网格 |
| `src/wheel_robot_moveit_config` | MoveIt 2 配置、ros2_control、RViz 和 launch |
| `src/wheel_robot_nav` | 轮速里程计、16 线定位、舵轮控制器和 Nav2 |
| `wheel_robot_moveit.usd` | Isaac Sim 场景：机器人、物理、机械臂 Action Graph、激光和轮子话题 |
| `wheel_robot_suction.usd` | 同一台机器人的副本。双手 Robotiq 关闭，手腕上是吸盘 |
| `wheel_robot_suction_stack.usd` | 在吸盘场景上加托盘、固定取料台，以及一只纸盒。放下成功后才在原位生成下一只 |
| `wheel_robot_nav_world.usd` | 导航场景：同一台机器人，加上 IMU、围墙和障碍 |
| `assets/grippers/robotiq_85_gripper.usd` | 可单独安装到其他机器人上的 Robotiq 2F-85 |
| `assets/grippers/suction_cup.usd` | 独立吸盘场景。原点是安装法兰，密封面沿 +Z。Action Graph 控制开合 |

没有复制：`isaaclab/`、齿轮装配策略、UR5，以及 `mbot_with_wheels`。

## 环境

- Ubuntu，ROS 2 Humble，MoveIt 2
- `colcon`、`xacro`
- 导航还需要 `nav2_bringup`、`nav2_smac_planner` 和 `teleop_twist_keyboard`
- Isaac Sim 4.5，用来在仿真里执行运动（默认 `/home/cym/isaac-sim-4.5.0`，可用环境变量 `ISAAC_SIM` 覆盖）

Humble 上可安装：

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

请用系统 Python 编译，不要用 conda 或 Isaac Sim 自带的解释器。

## 编译

```bash
source /opt/ros/humble/setup.bash
cd /home/cym/isaac_sim_projects/wheel_robot_moveit
colcon build --symlink-install
source install/setup.bash
```

每个新终端都要先 source：

```bash
source /opt/ros/humble/setup.bash
source /home/cym/isaac_sim_projects/wheel_robot_moveit/install/setup.bash
```

## 只看模型（不需要 Isaac Sim）

发布 URDF、打开关节滑条和 RViz：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch wheel_robot_moveit_config rsp.launch.py
```

另开一个终端：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
ros2 run rviz2 rviz2
```

RViz 的 **Fixed Frame** 设为 `world`，再添加 **RobotModel** 和 **TF**。拖动滑条可以活动升降、腰部、双臂和夹爪指节。

展开 URDF：

```bash
xacro "$(ros2 pkg prefix robot_description)/share/robot_description/urdf/wheel_robot/wheel_robot_with_grippers.xacro"
```

源文件位置：

- 机体：`src/robot_description/urdf/wheel_robot/wheel_robot_body.urdf.xacro`
- 夹爪：`src/robot_description/urdf/wheel_robot/wheel_robot_grippers.urdf.xacro`
- 顶层 xacro：`src/robot_description/urdf/wheel_robot/wheel_robot_with_grippers.xacro`
- 网格：`src/robot_description/meshes/wheel_robot/` 和 `src/robot_description/meshes/ur5/`

## 在 RViz 里用 MoveIt

`demo.launch.py` 会启动 `robot_state_publisher`、`move_group`、RViz 和 MoveIt warehouse。ros2_control 使用 `topic_based_ros2_control/TopicBasedSystem`，话题是 `/isaac_joint_states` 和 `/isaac_joint_commands`。没有 Isaac 时仍可打开 MotionPlanning 做规划；真正执行轨迹需要这些话题上有仿真或真机。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheel_robot_moveit_config demo.launch.py
```

MotionPlanning 面板：

1. Planning Group：`left_arm`、`right_arm`、`torso`、`left_gripper` 或 `right_gripper`。
2. Goal State：`left_home` / `right_home`、`nav_tuck`、`left_open` / `left_close`、`right_open` / `right_close`。
3. 先 Plan；有关节状态时再 Execute。

RViz 的 **Fixed Frame** 是 `world`。不要和 `isaac_moveit.launch.py` 或导航 launch 同时开，它们各自会启动 `controller_manager` 或 `robot_state_publisher`。

也可以拆开启动：

```bash
ros2 launch wheel_robot_moveit_config rsp.launch.py
ros2 launch wheel_robot_moveit_config move_group.launch.py
ros2 launch wheel_robot_moveit_config moveit_rviz.launch.py
```

## Isaac Sim 场景

USD 是独立文件，直接从本工作空间打开即可，不必再找 `moveit2_UR5`。

| 文件 | 用途 |
| --- | --- |
| `wheel_robot_moveit.usd` | 手臂、躯干和夹爪，配合 MoveIt。Isaac 发布 `/isaac_joint_states`，订阅 `/isaac_joint_commands`。 |
| `wheel_robot_suction.usd` | 夹爪换成吸盘的副本，手臂关节与上面相同。Play 之后 `left_suction_cmd` / `right_suction_cmd`（`std_msgs/Bool`，`true` 吸附）控制开合。 |
| `wheel_robot_suction_stack.usd` | 吸盘场景加上 1200×1000 mm 托盘和固定取料台。配合 `ros2 run wheel_robot_stack stack_boxes`。 |
| `wheel_robot_nav_world.usd` | 同一台机器人，加上 IMU、16 线激光、围墙和障碍。给 Nav2 用，也给导航+MoveIt 合并界面用。 |
| `assets/grippers/robotiq_85_gripper.usd` | 一只 Robotiq 2F-85。原点是工具法兰。只驱动 `robotiq_85_left_knuckle_joint`（0° 到 46°）。 |
| `assets/grippers/suction_cup.usd` | 独立吸盘。原点是安装法兰，密封面沿 +Z。Play 之后用话题 `suction_cmd`（`std_msgs/Bool`）或属性 `suctionClosed` 控制开合。 |

默认 Isaac Sim 安装路径：

```bash
ISAAC_SIM=${ISAAC_SIM:-/home/cym/isaac-sim-4.5.0}
```

在图形界面里用 **File → Open** 打开场景，或用命令行：

```bash
cd /home/cym/isaac_sim_projects/wheel_robot_moveit
ISAAC_SIM=${ISAAC_SIM:-/home/cym/isaac-sim-4.5.0}
"$ISAAC_SIM/isaac-sim.sh" "$(pwd)/wheel_robot_moveit.usd"
```

场景加载后按下 **Play**。场景里的 ROS 2 桥会开始收发话题。整个 ROS 会话期间都要保持 Play。

## 和 Isaac Sim 一起运行

1. 在 Isaac Sim 中打开本目录的 `wheel_robot_moveit.usd`，按下 Play。
2. Isaac 在 `/isaac_joint_states` 发布关节状态，并订阅 `/isaac_joint_commands`。
3. 在本工作空间另开一个终端：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheel_robot_moveit_config isaac_moveit.launch.py
```

重新启动前先停掉上一次 launch，保证只有一个 `controller_manager`。

### 控制器

| 控制器 | 类型 | 关节 |
| --- | --- | --- |
| `left_arm_controller` | `FollowJointTrajectory` | `Link1_L_joint` … `Link7_L_joint` |
| `right_arm_controller` | `FollowJointTrajectory` | `Link1_R_joint` … `Link7_R_joint` |
| `torso_controller` | `FollowJointTrajectory` | `linear_Link`、`waist_pitch_Link`、`waist_yaw_Link` |
| `left_gripper_controller` | `GripperActionController` | `left_robotiq_85_left_knuckle_joint` |
| `right_gripper_controller` | `GripperActionController` | `right_robotiq_85_left_knuckle_joint` |
| `joint_state_broadcaster` | 关节状态 | 全部可用接口 |

八个轮子关节不交给 MoveIt，也不进 `ros2_control`，由舵轮控制器驱动。夹爪只规划主动指节，另一侧由 mimic 跟着动。张开是 `0`，闭合是 `0.7` rad。

### 规划组

| 规划组 | 连杆 / 关节 |
| --- | --- |
| `torso` | `linear_Link`、`waist_pitch_Link`、`waist_yaw_Link` |
| `left_arm` | 链 `waist_yaw_Link` → `tool0_L` |
| `right_arm` | 链 `waist_yaw_Link` → `tool0_R` |
| `left_gripper` | `left_robotiq_85_left_knuckle_joint` |
| `right_gripper` | `right_robotiq_85_left_knuckle_joint` |
| `left_arm_with_torso` | 躯干 + 左臂 |
| `right_arm_with_torso` | 躯干 + 右臂 |

运动学用 KDL。手臂命名状态有 `left_home`、`right_home`，以及收在身侧方便开车的 `nav_tuck`。

## 导航

在 Isaac Sim 中打开本目录的 `wheel_robot_nav_world.usd`，按下 Play，然后：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheel_robot_nav navigation.launch.py
```

定位是激光惯性扫描匹配（`lidar_localization`）。轮速给出 `vx`、`vy`，IMU 陀螺给出偏航角速度，用来预测下一帧位姿。16 线点云再对体素地图做点到面配准，修正 `x`、`y` 和航向。离地 0.25–1.7 m 的回波同时涂成占据栅格 `/map`。`map` → `odom` 为单位变换，`odom` → `base_link` 由这次配准发布。导航用 Nav2：全局规划是 Smac 2D A*，局部控制是全向 MPPI，代价地图直接订阅 `/points`。

不要和 `isaac_moveit.launch.py` 或 `demo.launch.py` 一起开。导航 launch 发布 `odom` → `base_link`，那两个 MoveIt launch 使用固定的 `world` 连杆。

键盘试车：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

先让机器人转一小圈，把围墙扫进 `/map`，再在 RViz 里对已经扫到的区域点 Nav2 Goal。导航时升降保持在下限，雷达就一直在双臂上方。

### 话题

| 话题 | 方向 | 作用 |
| --- | --- | --- |
| `/isaac_joint_states` | Isaac → ROS | 全部关节，供里程计和 `robot_state_publisher` |
| `/isaac_joint_commands` | ROS → Isaac | 手臂、躯干和夹爪的位置指令 |
| `/isaac_wheel_steer` | ROS → Isaac | 四个转向关节的位置 |
| `/isaac_wheel_drive` | ROS → Isaac | 四个驱动关节的速度 |
| `/odom` | 轮速里程计 | 轮速积分，不发布 TF |
| `/imu` | Isaac | `imu_link` 上的 IMU，坐标系与 `base_link` 对齐 |
| `/odometry/filtered` | 16 线定位 | 预测加配准后的位姿，发布 `odom` → `base_link` |
| `/map` | 16 线定位 | 由多线回波涂出的占据栅格 |
| `/points` | Isaac | `lidar_link` 上的 16 线点云 |
| `/ground_truth_odom` | Isaac | 仿真真值，不发布 TF |
| `/cmd_vel` | Nav2 → 转向控制器 | 底盘速度 `vx, vy, wz` |

## 导航和 MoveIt 同一个界面

零位时两条手臂向左右伸出约 1 m，底盘能过去的缝，小臂过不去。合并 launch 用 MoveIt 把双臂收到身侧，代价地图的足迹也加宽到能包住收好的手臂。规划坐标系是 `odom`，和定位发布的 `odom` → `base_link` 接在一起，RViz 的 Fixed Frame 仍是 `map`。

Isaac Sim 要播放本目录的 `wheel_robot_nav_world.usd`，不要开 `wheel_robot_moveit.usd`。先停掉正在跑的 `navigation.launch.py` 或 `isaac_moveit.launch.py`。底盘要停在空地上再启动，手臂会从张开摆到身侧：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheel_robot_nav navigation_moveit.launch.py
```

等终端打出 `both arms are tucked` 再发导航目标。RViz 里 **Nav2 Goal** 发给底盘，**Interact** 拖动末端。MotionPlanning 面板里选 `left_arm` 或 `right_arm`，目标状态选 `nav_tuck`，再 Plan & Execute。

已经夹在障碍中间时不要直接收臂，先关掉收臂把车开到空地，再执行：

```bash
ros2 launch wheel_robot_nav navigation_moveit.launch.py tuck_arms:=false
ros2 run wheel_robot_nav tuck_arms --ros-args -p use_sim_time:=true
```

## `wheel_robot_nav` 里的节点

| 可执行文件 | 作用 |
| --- | --- |
| `wheel_odometry` | 把轮速积成 `/odom`（不发 TF） |
| `lidar_localization` | 用轮速和 IMU 预测，再用 `/points` 配准，发布 `odom` → `base_link` 和 `/map` |
| `swerve_controller` | 把 `/cmd_vel` 转成 `/isaac_wheel_steer` 和 `/isaac_wheel_drive` |
| `tuck_arms` | 先闭合夹爪，再规划并执行双臂 `nav_tuck` |
| `odom_fusion` | 额外的 EKF 辅助节点，默认 launch 不启动 |
| `imu_covariance` | 额外的 IMU 辅助节点，默认 launch 不启动 |

## 吸盘堆叠

打开 `wheel_robot_suction_stack.usd`，按 Play，并保持播放。另开终端先启动 MoveIt，再跑堆叠节点：

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

格子来自 Jylänki 2010 的 Maximal Rectangles / Bottom-Left：351×287 mm 的面在 1200×1000 mm 托盘上是每层 3×3，纸盒高 105 mm，1080 mm 里放 10 层，一共 90 格。托盘用 `assets/grippers/pallet.usd`，缩放到 1200×1000×125 mm，底面放在地面上。左臂水平够到的是靠近肩膀的一排，每层 3 只，共 30 只。取料点始终是 `(0.55, 0.18)`，取料台高 0.35 m。取料仍在原地完成。托盘上的格子超出手臂范围时，底盘会开到该格前方再放下，然后开回取料点。升降由躯干控制器单独执行。一只放稳之后，场景才在原位生成下一只。左吸盘话题是 `left_suction_cmd`。只打印格子、不动机器人：`ros2 run wheel_robot_stack stack_boxes --ros-args -p execute:=false`。

## 其他

- `colcon build` 必须用 Humble 的系统 Python。conda 或 Isaac Sim 的解释器找不到 `ament` / `rclpy`。
- 改了 URDF 或 SRDF 后要停掉旧 launch。用 `--symlink-install` 编译后，新终端再 `source install/setup.bash`。
- 独立吸盘在 `assets/grippers/suction_cup.usd`，不装在这台机器人上。Play 之后向 `suction_cmd` 发 `std_msgs/Bool`（`true` 吸附），或改 prim 上的 `suctionClosed`。物体需要刚体和碰撞，密封面大约 2 cm 以内才会吸住。
- 齿轮装配训练、`gear_demo` 和 `wheel_robot_gear_demo.usd` 仍在 `moveit2_UR5` 里，本工作空间没有。
