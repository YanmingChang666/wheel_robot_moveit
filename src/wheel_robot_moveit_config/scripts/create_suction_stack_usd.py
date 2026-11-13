#!/usr/bin/env python3
"""Copy wheel_robot_suction.usd and add the pallet, the pickup stand, and box 0.

Does not edit wheel_robot_suction.usd. The Action Graph spawns the next box at
the same pickup pose when stack_spawn increases, then publishes stack_box_ready.
"""

import shutil
import sys

sys.path.insert(0, "/home/cym/isaac_sim_projects/wheel_robot_moveit/src/wheel_robot_stack")

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.graph.core as og
import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics
from wheel_robot_stack.pattern import (
    BOX,
    STAND_SIZE,
    pallet_center,
    pickup_center,
    spawn_center,
    stand_center,
)

ROOT = "/home/cym/isaac_sim_projects/wheel_robot_moveit"
SOURCE = ROOT + "/wheel_robot_suction.usd"
OUT = ROOT + "/wheel_robot_suction_stack.usd"
STATUS = "/tmp/suction_stack_status.txt"

SPAWN_SCRIPT = f'''import omni.graph.core as og
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

PICKUP = {spawn_center()}
SIZE = {BOX}
READY = 0
# The subscriber keeps the last command after Stop. Treating that leftover
# as a new request spawns box_1 on top of box_0 as soon as Play starts.
BASELINE = None


def _read_spawn():
    node = og.Controller.node("/ActionGraph/stack_spawn_sub")
    if not node.is_valid():
        return None
    data = node.get_attribute("outputs:data")
    if not data.is_valid():
        return None
    return int(data.get())


def _publish_ready():
    node = og.Controller.node("/ActionGraph/stack_ready_pub")
    if not node.is_valid():
        return
    data = node.get_attribute("inputs:data")
    if not data.is_valid():
        return
    try:
        data.set(int(READY))
    except Exception:
        return


def _publish_poses():
    # World center of every box prim. The planner treats these as the real
    # obstacles; the nominal slot is only the target before the box settles.
    stage = omni.usd.get_context().get_stage()
    pub = og.Controller.node("/ActionGraph/stack_pose_pub")
    if stage is None or not pub.is_valid():
        return
    data = pub.get_attribute("inputs:data")
    if not data.is_valid():
        return
    parts = []
    index = 0
    while index < 40:
        prim = stage.GetPrimAtPath(f"/World/stack/box_{{index}}")
        if not prim.IsValid():
            break
        transform = Gf.Transform(omni.usd.get_world_transform_matrix(prim))
        translation = transform.GetTranslation()
        rotation = transform.GetRotation().GetQuat()
        imaginary = rotation.GetImaginary()
        parts.append(
            f"{{index}}:{{translation[0]:.4f}},{{translation[1]:.4f}},{{translation[2]:.4f}},"
            f"{{imaginary[0]:.5f}},{{imaginary[1]:.5f}},{{imaginary[2]:.5f}},{{rotation.GetReal():.5f}}"
        )
        index += 1
    # Negative IDs are actual world-frame gripper query poses, not boxes.
    for key, side in ((-1, "L"), (-2, "R")):
        prim = stage.GetPrimAtPath(f"/wheel_robot/tool0_{{side}}/suction_cup/grip_point")
        if not prim.IsValid():
            continue
        transform = Gf.Transform(omni.usd.get_world_transform_matrix(prim))
        t = transform.GetTranslation()
        q = transform.GetRotation().GetQuat()
        v = q.GetImaginary()
        parts.append(f"{{key}}:{{t[0]:.5f}},{{t[1]:.5f}},{{t[2]:.5f}},{{v[0]:.6f}},{{v[1]:.6f}},{{v[2]:.6f}},{{q.GetReal():.6f}}")
    try:
        data.set(";".join(parts))
    except Exception:
        return


def _spawn(index):
    stage = omni.usd.get_context().get_stage()
    path = f"/World/stack/box_{{index}}"
    if stage.GetPrimAtPath(path).IsValid():
        return
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*PICKUP))
    cube.AddScaleOp().Set(Gf.Vec3d(*SIZE))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.72, 0.55, 0.32)])
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.4)


def compute(db):
    global READY, BASELINE
    requested = _read_spawn()
    if BASELINE is None:
        BASELINE = requested
    elif requested is not None and requested != BASELINE:
        if requested > READY:
            _spawn(requested)
            READY = requested
        BASELINE = requested
    _publish_ready()
    _publish_poses()
    return True
'''


def _status(text):
    with open(STATUS, "a", encoding="utf-8") as handle:
        handle.write(text + "\n")


# Bounding box of assets/grippers/pallet.usd after its payload loads, in meters.
# Scale maps that box onto 1.200 x 1.000 x 0.125 with the bottom on z = 0.
_PALLET_MIN = (-0.6050764711556624, -0.3999999910593033, 1.3974263626320634e-7)
_PALLET_MAX = (0.6081584413016188, 0.4022984834395687, 0.14250582614900562)
_PALLET_TARGET = (1.200, 1.000, 0.125)


def _place_pallet(stage):
    path = "/World/stack/pallet"
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(ROOT + "/assets/grippers/pallet.usd")
    size = tuple(high - low for low, high in zip(_PALLET_MIN, _PALLET_MAX))
    scale = tuple(target / length for target, length in zip(_PALLET_TARGET, size))
    # This prim applies scale outside the translate, so the authored translation
    # is multiplied by the scale. Divide the desired shift by the scale.
    local_center = tuple((low + high) * 0.5 for low, high in zip(_PALLET_MIN, _PALLET_MAX))
    scaled_center = tuple(value * factor for value, factor in zip(local_center, scale))
    goal = pallet_center()
    translate = tuple((wanted - current) / factor for wanted, current, factor in zip(goal, scaled_center, scale))
    xform = UsdGeom.Xformable(prim)
    xform.AddScaleOp().Set(Gf.Vec3d(*scale))
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))


def _cube(stage, path, center, size, color, dynamic):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3d(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.4)
    return cube


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


def _build(stage):
    if stage.GetPrimAtPath("/World/stack").IsValid():
        stage.RemovePrim("/World/stack")
    UsdGeom.Xform.Define(stage, "/World/stack")
    _place_pallet(stage)
    _cube(stage, "/World/stack/stand", stand_center(), STAND_SIZE, (0.35, 0.35, 0.38), False)
    _cube(stage, "/World/stack/box_0", spawn_center(), BOX, (0.72, 0.55, 0.32), True)

    for name in ("stack_spawn_sub", "stack_ready_pub", "stack_pose_pub", "stack_spawner"):
        path = "/ActionGraph/" + name
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)

    keys = og.Controller.Keys
    og.Controller.edit(
        "/ActionGraph",
        {
            keys.CREATE_NODES: [
                ("stack_spawn_sub", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("stack_ready_pub", "isaacsim.ros2.bridge.ROS2Publisher"),
                ("stack_pose_pub", "isaacsim.ros2.bridge.ROS2Publisher"),
                ("stack_spawner", "omni.graph.scriptnode.ScriptNode"),
            ],
            keys.SET_VALUES: [
                ("stack_spawn_sub.inputs:messagePackage", "std_msgs"),
                ("stack_spawn_sub.inputs:messageSubfolder", "msg"),
                ("stack_spawn_sub.inputs:messageName", "Int32"),
                ("stack_spawn_sub.inputs:topicName", "stack_spawn"),
                ("stack_ready_pub.inputs:messagePackage", "std_msgs"),
                ("stack_ready_pub.inputs:messageSubfolder", "msg"),
                ("stack_ready_pub.inputs:messageName", "Int32"),
                ("stack_ready_pub.inputs:topicName", "stack_box_ready"),
                ("stack_pose_pub.inputs:messagePackage", "std_msgs"),
                ("stack_pose_pub.inputs:messageSubfolder", "msg"),
                ("stack_pose_pub.inputs:messageName", "String"),
                ("stack_pose_pub.inputs:topicName", "stack_box_poses"),
                ("stack_spawner.inputs:usePath", False),
                ("stack_spawner.inputs:script", SPAWN_SCRIPT),
            ],
        },
    )
    for _ in range(3):
        simulation_app.update()
    command = og.Controller.node("/ActionGraph/stack_spawner")
    initialized = command.get_attribute("state:omni_initialized")
    if initialized.is_valid():
        initialized.set(False)
    links = [
        ("/ActionGraph/on_playback_tick.outputs:tick", "/ActionGraph/stack_spawn_sub.inputs:execIn"),
        ("/ActionGraph/on_playback_tick.outputs:tick", "/ActionGraph/stack_ready_pub.inputs:execIn"),
        ("/ActionGraph/on_playback_tick.outputs:tick", "/ActionGraph/stack_pose_pub.inputs:execIn"),
        ("/ActionGraph/on_playback_tick.outputs:tick", "/ActionGraph/stack_spawner.inputs:execIn"),
        ("/ActionGraph/ros2_context.outputs:context", "/ActionGraph/stack_spawn_sub.inputs:context"),
        ("/ActionGraph/ros2_context.outputs:context", "/ActionGraph/stack_ready_pub.inputs:context"),
        ("/ActionGraph/ros2_context.outputs:context", "/ActionGraph/stack_pose_pub.inputs:context"),
    ]
    for source, destination in links:
        _connect(stage, source, destination)


def main():
    open(STATUS, "w", encoding="utf-8").close()
    shutil.copy(SOURCE, OUT)
    manager = omni.kit.app.get_app().get_extension_manager()
    for name in ("isaacsim.robot.surface_gripper", "isaacsim.ros2.bridge"):
        manager.set_extension_enabled_immediate(name, True)
    for _ in range(5):
        simulation_app.update()
    context = omni.usd.get_context()
    context.open_stage(OUT)
    for _ in range(8):
        simulation_app.update()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("failed to open " + OUT)
    _build(stage)
    for _ in range(3):
        simulation_app.update()
    if not context.save_stage():
        raise RuntimeError("failed to save " + OUT)
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
