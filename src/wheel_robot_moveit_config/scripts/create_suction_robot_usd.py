#!/usr/bin/env python3
"""Copy wheel_robot_moveit.usd and replace both grippers with suction cups.

The original scenes are only read. Output is wheel_robot_suction.usd.
Robotiq prims in the copy are deactivated. Each cup is a child of tool0.
/ActionGraph steps the Surface Gripper every tick and pulses Close or Open
when left_suction_cmd / right_suction_cmd or suctionClosed changes.
"""

import math
import shutil
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.graph.core as og
import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

ROOT = "/home/cym/isaac_sim_projects/wheel_robot_moveit"
SOURCE = ROOT + "/wheel_robot_moveit.usd"
OUT = ROOT + "/wheel_robot_suction.usd"
STATUS = "/tmp/suction_robot_status.txt"

# Gripper local +X is the suction axis. Ry(-90) maps that axis onto tool0 +Z.
GRIP_ORIENT = Gf.Quatf(math.cos(-math.pi / 4.0), 0.0, math.sin(-math.pi / 4.0), 0.0)

SUCTION_SCRIPT = '''import omni.graph.core as og
import omni.usd

PREVIOUS = {}

ARMS = (
    ("left", "/ActionGraph/suction_left_sub", "/wheel_robot/tool0_L/suction_cup",
     "/ActionGraph/suction_left_close", "/ActionGraph/suction_left_open"),
    ("right", "/ActionGraph/suction_right_sub", "/wheel_robot/tool0_R/suction_cup",
     "/ActionGraph/suction_right_close", "/ActionGraph/suction_right_open"),
)


def _pulse(path):
    node = og.Controller.node(path)
    if not node.is_valid():
        return
    impulse = node.get_attribute("state:enableImpulse")
    if impulse.is_valid():
        impulse.set(True)
    node.request_compute()


def _ros_closed(node_path):
    node = og.Controller.node(node_path)
    if not node.is_valid():
        return None
    data = node.get_attribute("outputs:data")
    if not data.is_valid():
        return None
    return bool(data.get())


def _prim_closed(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    attr = prim.GetAttribute("suctionClosed")
    if not attr.IsValid() or not attr.HasAuthoredValue():
        return None
    return bool(attr.Get())


def _on_change(key, value, close_path, open_path):
    if value is None:
        return
    seen = key in PREVIOUS
    last = PREVIOUS.get(key)
    PREVIOUS[key] = value
    if not seen or last == value:
        return
    _pulse(close_path if value else open_path)


# onStep must come from an OnImpulseEvent. When a grip lets go, the
# SurfaceGripper writes state:enableImpulse on whatever feeds onStep.
# on_playback_tick has no such attribute, so the second grip aborts.
STEP = (
    "/ActionGraph/suction_left_step",
    "/ActionGraph/suction_right_step",
)


def _publish_holding():
    # outputs:Closed is true only while the cup is actually holding a body.
    # The command topic stays true even when the seal missed, which is what
    # let the arm place an empty cup and stack the next box on air.
    pairs = (
        ("/ActionGraph/suction_left", "/ActionGraph/suction_holding_pub"),
        ("/ActionGraph/suction_right", "/ActionGraph/suction_holding_pub_right"),
    )
    for grip_path, pub_path in pairs:
        grip = og.Controller.node(grip_path)
        pub = og.Controller.node(pub_path)
        if not grip.is_valid() or not pub.is_valid():
            continue
        closed = grip.get_attribute("outputs:Closed")
        data = pub.get_attribute("inputs:data")
        if not closed.is_valid() or not data.is_valid():
            continue
        try:
            data.set(bool(closed.get()))
        except Exception:
            continue


def compute(db):
    for path in STEP:
        _pulse(path)
    for key, subscriber, prim_path, close_path, open_path in ARMS:
        _on_change(key + "_ros", _ros_closed(subscriber), close_path, open_path)
        _on_change(key + "_prim", _prim_closed(prim_path), close_path, open_path)
    _publish_holding()
    return True
'''


def _status(text):
    with open(STATUS, "a", encoding="utf-8") as handle:
        handle.write(text + "\n")


def _enable_extensions():
    manager = omni.kit.app.get_app().get_extension_manager()
    for name in ("isaacsim.robot.surface_gripper", "isaacsim.ros2.bridge"):
        manager.set_extension_enabled_immediate(name, True)
    for _ in range(5):
        simulation_app.update()


def _cylinder(stage, path, radius, height, z_center, color):
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(height)
    cylinder.CreateAxisAttr("Z")
    cylinder.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z_center))
    cylinder.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    return cylinder


def _add_cup(stage, tool_path):
    cup_path = tool_path + "/suction_cup"
    existing = stage.GetPrimAtPath(cup_path)
    if existing.IsValid():
        stage.RemovePrim(cup_path)
    cup = UsdGeom.Xform.Define(stage, cup_path)
    cup.GetPrim().CreateAttribute("suctionClosed", Sdf.ValueTypeNames.Bool).Set(False)
    _cylinder(stage, cup_path + "/neck", 0.012, 0.04, 0.02, (0.75, 0.75, 0.78))
    _cylinder(stage, cup_path + "/lip", 0.028, 0.008, 0.044, (0.9, 0.4, 0.1))
    grip = UsdGeom.Xform.Define(stage, cup_path + "/grip_point")
    grip.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.051))
    grip.AddOrientOp().Set(GRIP_ORIENT)
    return cup_path + "/grip_point"


def _deactivate_robotiq(stage):
    robot = stage.GetPrimAtPath("/wheel_robot")
    if not robot.IsValid():
        raise RuntimeError("missing /wheel_robot")
    for child in robot.GetChildren():
        if "robotiq" in child.GetName():
            child.SetActive(False)
    for joint_path in (
        "/wheel_robot/tool0_L/left_robotiq_coupler_joint",
        "/wheel_robot/tool0_R/right_robotiq_coupler_joint",
    ):
        joint = stage.GetPrimAtPath(joint_path)
        if joint.IsValid():
            joint.SetActive(False)


def _connect(stage, source, destination):
    try:
        og.Controller.connect(source, destination)
        return
    except Exception as exc:
        _status(f"og connect failed {destination} {exc}")
    dest_prim_path, dest_attr = destination.rsplit(".", 1)
    src_prim_path, src_attr_name = source.rsplit(".", 1)
    src_attr = stage.GetPrimAtPath(src_prim_path).GetAttribute(src_attr_name)
    prim = stage.GetPrimAtPath(dest_prim_path)
    attr = prim.GetAttribute(dest_attr)
    if not attr:
        value_type = src_attr.GetTypeName() if src_attr else Sdf.ValueTypeNames.UInt
        attr = prim.CreateAttribute(dest_attr, value_type, custom=True)
    attr.AddConnection(Sdf.Path(source))


# MoveIt still commands the Robotiq knuckle joints. Those DOFs are gone with the
# grippers. Isaac drops the whole command when any name is missing, so the arm
# never moves unless these two names are removed first.
FILTER_SCRIPT = '''import omni.graph.core as og

DROP = (
    "left_robotiq_85_left_knuckle_joint",
    "right_robotiq_85_left_knuckle_joint",
)


def compute(db):
    sub = og.Controller.node("/ActionGraph/ros2_subscribe_joint_state")
    ctrl = og.Controller.node("/ActionGraph/articulation_controller")
    if not sub.is_valid() or not ctrl.is_valid():
        return True
    def as_list(value):
        if value is None:
            return []
        return list(value)

    names = as_list(og.Controller.get(sub.get_attribute("outputs:jointNames")))
    if len(names) == 0:
        return True
    pos = as_list(og.Controller.get(sub.get_attribute("outputs:positionCommand")))
    vel = as_list(og.Controller.get(sub.get_attribute("outputs:velocityCommand")))
    eff = as_list(og.Controller.get(sub.get_attribute("outputs:effortCommand")))
    keep = [i for i, name in enumerate(names) if name not in DROP]

    def take(values):
        if len(values) != len(names):
            return None
        return [values[i] for i in keep]

    og.Controller.set(ctrl.get_attribute("inputs:jointNames"), [names[i] for i in keep])
    positions = take(pos)
    velocities = take(vel)
    efforts = take(eff)
    if positions is not None:
        og.Controller.set(ctrl.get_attribute("inputs:positionCommand"), positions)
    if velocities is not None:
        og.Controller.set(ctrl.get_attribute("inputs:velocityCommand"), velocities)
    if efforts is not None:
        og.Controller.set(ctrl.get_attribute("inputs:effortCommand"), efforts)
    return True
'''


def _filter_gripper_commands(stage):
    """Run the arm command only after the two missing knuckle names are removed."""
    path = "/ActionGraph/filter_arm_command"
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)
    node = stage.DefinePrim(path, "OmniGraphNode")
    node.CreateAttribute("node:type", Sdf.ValueTypeNames.Token).Set("omni.graph.scriptnode.ScriptNode")
    node.CreateAttribute("node:typeVersion", Sdf.ValueTypeNames.Int).Set(2)
    node.CreateAttribute("inputs:usePath", Sdf.ValueTypeNames.Bool).Set(False)
    node.CreateAttribute("inputs:script", Sdf.ValueTypeNames.String).Set(FILTER_SCRIPT)
    node.CreateAttribute("inputs:execIn", Sdf.ValueTypeNames.UInt)
    node.CreateAttribute("outputs:execOut", Sdf.ValueTypeNames.UInt)
    node.CreateAttribute("state:omni_initialized", Sdf.ValueTypeNames.Bool).Set(False)

    controller = stage.GetPrimAtPath("/ActionGraph/articulation_controller")
    for name in (
        "inputs:execIn",
        "inputs:jointNames",
        "inputs:positionCommand",
        "inputs:velocityCommand",
        "inputs:effortCommand",
    ):
        attr = controller.GetAttribute(name)
        if attr:
            attr.ClearConnections()
    _connect(stage, "/ActionGraph/ros2_subscribe_joint_state.outputs:execOut", path + ".inputs:execIn")
    _connect(stage, path + ".outputs:execOut", "/ActionGraph/articulation_controller.inputs:execIn")


def _graph(stage):
    if not stage.GetPrimAtPath("/ActionGraph/on_playback_tick").IsValid():
        raise RuntimeError("missing /ActionGraph/on_playback_tick")
    if not stage.GetPrimAtPath("/ActionGraph/ros2_context").IsValid():
        raise RuntimeError("missing /ActionGraph/ros2_context")

    stale = [
        "suction_left",
        "suction_right",
        "suction_left_close",
        "suction_left_open",
        "suction_right_close",
        "suction_right_open",
        "suction_left_sub",
        "suction_right_sub",
        "suction_holding_pub",
        "suction_holding_pub_right",
        "suction_command",
    ]
    for name in stale:
        path = "/ActionGraph/" + name
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)

    keys = og.Controller.Keys
    og.Controller.edit(
        "/ActionGraph",
        {
            keys.CREATE_NODES: [
                ("suction_left", "isaacsim.robot.surface_gripper.SurfaceGripper"),
                ("suction_right", "isaacsim.robot.surface_gripper.SurfaceGripper"),
                ("suction_left_close", "omni.graph.action.OnImpulseEvent"),
                ("suction_left_open", "omni.graph.action.OnImpulseEvent"),
                ("suction_left_step", "omni.graph.action.OnImpulseEvent"),
                ("suction_right_close", "omni.graph.action.OnImpulseEvent"),
                ("suction_right_open", "omni.graph.action.OnImpulseEvent"),
                ("suction_right_step", "omni.graph.action.OnImpulseEvent"),
                ("suction_left_sub", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("suction_right_sub", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("suction_holding_pub", "isaacsim.ros2.bridge.ROS2Publisher"),
                ("suction_holding_pub_right", "isaacsim.ros2.bridge.ROS2Publisher"),
                ("suction_command", "omni.graph.scriptnode.ScriptNode"),
            ],
            keys.SET_VALUES: [
                ("suction_left.inputs:GripThreshold", 0.02),
                ("suction_left.inputs:ForceLimit", 500.0),
                ("suction_left.inputs:TorqueLimit", 80.0),
                ("suction_left.inputs:BendAngle", 15.0),
                ("suction_left.inputs:Stiffness", 10000.0),
                ("suction_left.inputs:Damping", 1000.0),
                ("suction_left.inputs:DisableGravity", True),
                ("suction_left.inputs:RetryClose", False),
                ("suction_right.inputs:GripThreshold", 0.02),
                ("suction_right.inputs:ForceLimit", 500.0),
                ("suction_right.inputs:TorqueLimit", 80.0),
                ("suction_right.inputs:BendAngle", 15.0),
                ("suction_right.inputs:Stiffness", 10000.0),
                ("suction_right.inputs:Damping", 1000.0),
                ("suction_right.inputs:DisableGravity", True),
                ("suction_right.inputs:RetryClose", False),
                ("suction_left_sub.inputs:messagePackage", "std_msgs"),
                ("suction_left_sub.inputs:messageSubfolder", "msg"),
                ("suction_left_sub.inputs:messageName", "Bool"),
                ("suction_left_sub.inputs:topicName", "left_suction_cmd"),
                ("suction_right_sub.inputs:messagePackage", "std_msgs"),
                ("suction_right_sub.inputs:messageSubfolder", "msg"),
                ("suction_right_sub.inputs:messageName", "Bool"),
                ("suction_right_sub.inputs:topicName", "right_suction_cmd"),
                ("suction_holding_pub.inputs:messagePackage", "std_msgs"),
                ("suction_holding_pub.inputs:messageSubfolder", "msg"),
                ("suction_holding_pub.inputs:messageName", "Bool"),
                ("suction_holding_pub.inputs:topicName", "left_suction_holding"),
                ("suction_holding_pub_right.inputs:messagePackage", "std_msgs"),
                ("suction_holding_pub_right.inputs:messageSubfolder", "msg"),
                ("suction_holding_pub_right.inputs:messageName", "Bool"),
                ("suction_holding_pub_right.inputs:topicName", "right_suction_holding"),
                ("suction_command.inputs:usePath", False),
                ("suction_command.inputs:script", SUCTION_SCRIPT),
            ],
        },
    )
    for _ in range(3):
        simulation_app.update()
    links = [
        ("/ActionGraph/suction_left_step.outputs:execOut", "/ActionGraph/suction_left.inputs:onStep"),
        ("/ActionGraph/suction_right_step.outputs:execOut", "/ActionGraph/suction_right.inputs:onStep"),
        ("/ActionGraph/on_playback_tick.outputs:tick", "/ActionGraph/suction_left_sub.inputs:execIn"),
        ("/ActionGraph/on_playback_tick.outputs:tick", "/ActionGraph/suction_right_sub.inputs:execIn"),
        ("/ActionGraph/on_playback_tick.outputs:tick", "/ActionGraph/suction_command.inputs:execIn"),
        ("/ActionGraph/on_playback_tick.outputs:tick", "/ActionGraph/suction_holding_pub.inputs:execIn"),
        ("/ActionGraph/on_playback_tick.outputs:tick", "/ActionGraph/suction_holding_pub_right.inputs:execIn"),
        ("/ActionGraph/ros2_context.outputs:context", "/ActionGraph/suction_left_sub.inputs:context"),
        ("/ActionGraph/ros2_context.outputs:context", "/ActionGraph/suction_right_sub.inputs:context"),
        ("/ActionGraph/ros2_context.outputs:context", "/ActionGraph/suction_holding_pub.inputs:context"),
        ("/ActionGraph/ros2_context.outputs:context", "/ActionGraph/suction_holding_pub_right.inputs:context"),
        ("/ActionGraph/suction_left_close.outputs:execOut", "/ActionGraph/suction_left.inputs:Close"),
        ("/ActionGraph/suction_left_open.outputs:execOut", "/ActionGraph/suction_left.inputs:Open"),
        ("/ActionGraph/suction_right_close.outputs:execOut", "/ActionGraph/suction_right.inputs:Close"),
        ("/ActionGraph/suction_right_open.outputs:execOut", "/ActionGraph/suction_right.inputs:Open"),
    ]
    for source, destination in links:
        _connect(stage, source, destination)
    for side, tool in (("left", "/wheel_robot/tool0_L"), ("right", "/wheel_robot/tool0_R")):
        node = stage.GetPrimAtPath(f"/ActionGraph/suction_{side}")
        body = node.GetRelationship("inputs:ParentRigidBody")
        if not body:
            body = node.CreateRelationship("inputs:ParentRigidBody")
        body.SetTargets([Sdf.Path(tool)])
        point = node.GetRelationship("inputs:GripPosition")
        if not point:
            point = node.CreateRelationship("inputs:GripPosition")
        point.SetTargets([Sdf.Path(tool + "/suction_cup/grip_point")])
    _filter_gripper_commands(stage)


def main():
    open(STATUS, "w", encoding="utf-8").close()
    shutil.copy(SOURCE, OUT)
    _status("copied")
    _enable_extensions()
    context = omni.usd.get_context()
    context.open_stage(OUT)
    for _ in range(8):
        simulation_app.update()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError(f"failed to open {OUT}")
    _deactivate_robotiq(stage)
    _add_cup(stage, "/wheel_robot/tool0_L")
    _add_cup(stage, "/wheel_robot/tool0_R")
    _graph(stage)
    for _ in range(3):
        simulation_app.update()
    if not context.save_stage():
        raise RuntimeError(f"failed to save {OUT}")
    _status("saved " + OUT)
    return 0


if __name__ == "__main__":
    try:
        code = main()
    except Exception as exc:
        _status("ERROR " + repr(exc))
        code = 1
    simulation_app.close()
    sys.exit(code)
