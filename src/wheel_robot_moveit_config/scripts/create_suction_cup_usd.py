#!/usr/bin/env python3
"""Write assets/grippers/suction_cup.usd. Does not edit the robot stages.

Run with Isaac Sim's python.sh. The stage is a standalone suction cup:
flange at the cup prim origin, seal along local +Z, Surface Gripper driven
by an Action Graph. A dynamic box sits under the seal so Play plus close
can weld it. Open and close come from the ROS topic suction_cmd
(std_msgs/Bool, true grips) or the suctionClosed attribute on the cup.
"""

import math
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.graph.core as og
import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

OUT = "/home/cym/isaac_sim_projects/wheel_robot_moveit/assets/grippers/suction_cup.usd"
STATUS = "/tmp/suction_cup_status.txt"
CUP = "/World/suction_cup"
# Gripper local +X is the suction axis. Ry(-90) maps it onto cup +Z.
GRIP_ORIENT = Gf.Quatf(math.cos(-math.pi / 4.0), 0.0, math.sin(-math.pi / 4.0), 0.0)

SUCTION_SCRIPT = '''import omni.graph.core as og
import omni.usd

PREVIOUS = {}
SUBSCRIBER = "/ActionGraph/suction_sub"
PRIM = "/World/suction_cup"
CLOSE = "/ActionGraph/suction_close"
OPEN = "/ActionGraph/suction_open"


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


def compute(db):
    _on_change("ros", _ros_closed(SUBSCRIBER), CLOSE, OPEN)
    _on_change("prim", _prim_closed(PRIM), CLOSE, OPEN)
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


def _build_stage():
    context = omni.usd.get_context()
    context.new_stage()
    for _ in range(3):
        simulation_app.update()
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    scene = UsdPhysics.Scene.Define(stage, "/World/physics_scene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)

    ground = UsdGeom.Cube.Define(stage, "/World/ground")
    ground.CreateSizeAttr(1.0)
    ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.05))
    ground.AddScaleOp().Set(Gf.Vec3d(2.0, 2.0, 0.1))
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.45, 0.45, 0.45)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    # Cup flange is the prim origin. The world lift puts the seal 16 mm
    # above a 60 mm box (inside the 20 mm grip threshold) with 5 mm clearance
    # under the lip.
    cup = UsdGeom.Xform.Define(stage, CUP)
    cup.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.025))
    cup_prim = cup.GetPrim()
    cup_prim.CreateAttribute("suctionClosed", Sdf.ValueTypeNames.Bool).Set(False)
    UsdPhysics.RigidBodyAPI.Apply(cup_prim)
    UsdPhysics.RigidBodyAPI(cup_prim).CreateKinematicEnabledAttr(True)
    _cylinder(stage, CUP + "/neck", 0.012, 0.04, 0.02, (0.75, 0.75, 0.78))
    _cylinder(stage, CUP + "/lip", 0.028, 0.008, 0.044, (0.9, 0.4, 0.1))
    grip = UsdGeom.Xform.Define(stage, CUP + "/grip_point")
    grip.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.051))
    grip.AddOrientOp().Set(GRIP_ORIENT)

    box = UsdGeom.Cube.Define(stage, "/World/box")
    box.CreateSizeAttr(1.0)
    box.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.03))
    box.AddScaleOp().Set(Gf.Vec3d(0.15, 0.12, 0.06))
    box.CreateDisplayColorAttr([Gf.Vec3f(0.72, 0.55, 0.32)])
    box_prim = box.GetPrim()
    UsdPhysics.CollisionAPI.Apply(box_prim)
    UsdPhysics.RigidBodyAPI.Apply(box_prim)
    UsdPhysics.MassAPI.Apply(box_prim).CreateMassAttr(0.3)
    return stage


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


def _graph(stage):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
                ("suction", "isaacsim.robot.surface_gripper.SurfaceGripper"),
                ("suction_close", "omni.graph.action.OnImpulseEvent"),
                ("suction_open", "omni.graph.action.OnImpulseEvent"),
                ("suction_sub", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("suction_command", "omni.graph.scriptnode.ScriptNode"),
            ],
            keys.SET_VALUES: [
                ("suction.inputs:GripThreshold", 0.02),
                ("suction.inputs:ForceLimit", 500.0),
                ("suction.inputs:TorqueLimit", 80.0),
                ("suction.inputs:BendAngle", 15.0),
                ("suction.inputs:Stiffness", 10000.0),
                ("suction.inputs:Damping", 1000.0),
                ("suction.inputs:DisableGravity", True),
                ("suction.inputs:RetryClose", False),
                ("suction_sub.inputs:messagePackage", "std_msgs"),
                ("suction_sub.inputs:messageSubfolder", "msg"),
                ("suction_sub.inputs:messageName", "Bool"),
                ("suction_sub.inputs:topicName", "suction_cmd"),
                ("suction_command.inputs:usePath", False),
                ("suction_command.inputs:script", SUCTION_SCRIPT),
            ],
        },
    )
    for _ in range(3):
        simulation_app.update()

    graph = stage.GetPrimAtPath("/ActionGraph")
    for name, value in (
        ("pipelineStage", "pipelineStageSimulation"),
        ("evaluationMode", "Automatic"),
    ):
        attr = graph.GetAttribute(name)
        if attr:
            attr.Set(value)

    command = og.Controller.node("/ActionGraph/suction_command")
    initialized = command.get_attribute("state:omni_initialized")
    if initialized.is_valid():
        initialized.set(False)

    links = [
        ("/ActionGraph/on_playback_tick.outputs:tick", "/ActionGraph/suction.inputs:onStep"),
        ("/ActionGraph/on_playback_tick.outputs:tick", "/ActionGraph/suction_sub.inputs:execIn"),
        ("/ActionGraph/on_playback_tick.outputs:tick", "/ActionGraph/suction_command.inputs:execIn"),
        ("/ActionGraph/ros2_context.outputs:context", "/ActionGraph/suction_sub.inputs:context"),
        ("/ActionGraph/suction_close.outputs:execOut", "/ActionGraph/suction.inputs:Close"),
        ("/ActionGraph/suction_open.outputs:execOut", "/ActionGraph/suction.inputs:Open"),
    ]
    for source, destination in links:
        _connect(stage, source, destination)

    node = stage.GetPrimAtPath("/ActionGraph/suction")
    body = node.GetRelationship("inputs:ParentRigidBody")
    if not body:
        body = node.CreateRelationship("inputs:ParentRigidBody")
    body.SetTargets([Sdf.Path(CUP)])
    point = node.GetRelationship("inputs:GripPosition")
    if not point:
        point = node.CreateRelationship("inputs:GripPosition")
    point.SetTargets([Sdf.Path(CUP + "/grip_point")])


def main():
    open(STATUS, "w", encoding="utf-8").close()
    _enable_extensions()
    stage = _build_stage()
    _graph(stage)
    for _ in range(3):
        simulation_app.update()
    stage.GetRootLayer().Export(OUT)
    _status("exported " + OUT)
    return 0


if __name__ == "__main__":
    try:
        code = main()
    except Exception as exc:
        _status("ERROR " + repr(exc))
        code = 1
    simulation_app.close()
    sys.exit(code)
