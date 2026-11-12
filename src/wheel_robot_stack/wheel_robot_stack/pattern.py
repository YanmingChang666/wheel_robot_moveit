"""Layer pallet pattern for identical boxes.

Each layer is packed with Maximal Rectangles and the Bottom-Left rule from
Jukka Jylänki, "A Thousand Ways to Pack the Bin — A Practical Approach to
Two-Dimensional Rectangle Bin Packing" (2010):
http://pds25.egloos.com/pds/201504/21/98/RectangleBinPack.pdf

The same heuristic is what the MIT-licensed rectpack library implements. This
module is a small reimplementation so the stack node has no extra dependency.
Identical 351 x 287 mm faces on a 1200 x 1000 mm pallet produce a 3 x 3 grid.
Layers repeat until the pile would pass 1080 mm (10 layers, 1050 mm).

Changes from the previous revision
----------------------------------
* ``base_for_slot`` no longer falls back silently. It searches the verified
  base-frame offsets (heading 0 by default, side headings via ``SIDE_YAWS``), requires the shoulder-to-slot reach to
  hold, and raises ``NoBasePose`` when nothing is feasible.
* ``pallet_slots`` keeps only slots that have a feasible base pose, instead of
  filtering with a fixed base at the origin. Within a layer it fills the far
  row first (largest y), so the arm never reaches over boxes just placed.
* ``tool_z_for_place`` is derived from the carried box center, so the box
  bottom lands on ``PLACE_CLEARANCE`` above the support. Before, the box was
  released about CONTACT_PRESS (18 mm) too high.
* ``_prune`` no longer deletes both copies of an identical free rectangle.
* ``lift_for_tool_z`` can be strict about the upper lift limit.
"""

import math
from dataclasses import dataclass
from functools import lru_cache


BOX = (0.351, 0.287, 0.105)
PALLET_FOOTPRINT = (1.200, 1.000)
PALLET_MAX_HEIGHT = 1.080
PALLET_THICKNESS = 0.125
# Min corner of the wooden pallet measured from wheel_robot_suction_stack.usd.
# The prim order is scale then translate, and the translate is scaled, so the
# deck is x = -0.5985 .. 0.6015, y = 0.6214 .. 1.6214, z = 0 .. 0.125.
# Center (0.002, 1.121, 0.063). This is the box RViz draws.
PALLET_ORIGIN = (-0.5985, 0.6214)
# A static plywood-style riser lying on the pallet deck. The arm cannot get the
# tool low enough for a layer sitting on a ground-level pallet (lift is already
# at its lower limit 0), so layer 0 is raised by this much. The simulation
# scene needs the same riser: run make_riser_overlay.py to generate it, and keep
# this number and the scene in step. Set it to 0.0 for no riser.
RISER_HEIGHT = 0.08
# Top of whatever the boxes rest on: pallet deck plus riser.
PALLET_TOP_Z = PALLET_THICKNESS + RISER_HEIGHT
# Pickup stand stays at the original bench height.
STAND_TOP_Z = 0.35
PICKUP_XY = (0.55, 0.18)
STAND_SIZE = (0.50, 0.36, 0.30)
# Left-arm shoulder at lift 0, from the wheel-robot URDF.
SHOULDER_XY = (0.13755, 0.1212)
# Horizontal distance the downward wrist can be expected to reach.
REACH_M = 0.85
# Cup geometry from create_suction_robot_usd._add_cup. The gold lip is a
# cylinder of height 0.008 centered at 0.044, so the sealing face is 0.048
# along tool +Z. The surface-gripper sample point is 0.051 and closes
# within 0.020.
LIP_FACE = 0.048
GRIP_POINT = 0.051
# The arm stops about 1 cm short. 18 mm of commanded press still leaves the
# lip in the cardboard after that error, inside the 20 mm grip distance.
CONTACT_PRESS = 0.018
# Gap between the carried box bottom and the support when the cup lets go.
# 0.0 puts the box exactly on the support. The old code effectively used
# CONTACT_PRESS here; set it to 0.018 to get the old release height back.
PLACE_CLEARANCE = 0.0
APPROACH_HEIGHT = 0.18
# Shoulder height at lift 0. The lift joint is commanded so the shoulder
# stays a little above the suction face.
SHOULDER_Z0 = 0.9105
LIFT_LIMIT = 0.7
# Lowest tool0 height the arm can be trusted to reach at lift 0 with the tool
# pointing down, near base-frame offset (0.09, 0.47). Two stack_boxes runs gave
# different limits: IK failed at z = 0.283 in one and succeeded down to 0.278
# (failed at 0.260) in the other, so the real limit is about 0.27-0.28 and moves
# by a centimetre or two with the base stop error. 0.30 keeps a margin.
# Re-measure it for your arm; the checker uses it to flag slots set too low.
MIN_TOOL_Z = 0.30

# Chassis footprint half extents in the base frame (x, y).
BASE_HALF = (0.26, 0.24)
# Extra gap kept between the chassis footprint and the pallet or stand. The
# footprint above is an estimate; swerve modules and wheels stick out, and the
# drive loop stops 4 cm short of its target at worst.
BASE_MARGIN = 0.05
# Slot position in the base frame that the downward tool was checked against
# at deck height. These are the verified offsets from the old base_for_slot.
# Widen them only after checking the arm really reaches there.
LOCAL_X_OPTIONS = (0.05, 0.12, 0.20, 0.30, 0.38)
LOCAL_Y_OPTIONS = (0.32, 0.36, 0.40, 0.45)
LOCAL_X_PREFERRED = 0.15
LOCAL_Y_PREFERRED = 0.42
# Headings tried for the chassis. Only heading 0 has been verified, so it is the
# default. SIDE_YAWS lets the chassis park beside the pallet, which reaches the
# left column of the far rows, but the node must then drive around the pallet
# instead of in a straight line.
BASE_YAWS = (0.0,)
SIDE_YAWS = (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)
YAW_PENALTY = 0.2


class NoBasePose(RuntimeError):
    """No chassis pose reaches the slot without touching the pallet or stand."""


@dataclass(frozen=True)
class Slot:
    index: int
    layer: int
    x: float
    y: float
    z: float
    rotated: bool

    @property
    def support_z(self):
        return PALLET_TOP_Z + self.layer * BOX[2]


# ---------------------------------------------------------------------------
# Layer packing (Maximal Rectangles, Bottom-Left)
# ---------------------------------------------------------------------------

def _contained(inner, outer):
    return (
        inner[0] >= outer[0] - 1e-9
        and inner[1] >= outer[1] - 1e-9
        and inner[0] + inner[2] <= outer[0] + outer[2] + 1e-9
        and inner[1] + inner[3] <= outer[1] + outer[3] + 1e-9
    )


def _prune(free):
    """Drop empty rectangles and ones inside another rectangle.

    Two identical rectangles contain each other, so the earlier one is kept
    and the later one dropped. Dropping both would lose free space.
    """
    kept = []
    for index, rect in enumerate(free):
        if rect[2] < 1e-6 or rect[3] < 1e-6:
            continue
        dominated = False
        for other_index, other in enumerate(free):
            if other_index == index or not _contained(rect, other):
                continue
            if _contained(other, rect) and index < other_index:
                continue  # identical copy; this one is the survivor
            dominated = True
            break
        if not dominated:
            kept.append(rect)
    return kept


def _split(free, px, py, pw, ph):
    remnants = []
    for x, y, w, h in free:
        if px >= x + w or px + pw <= x or py >= y + h or py + ph <= y:
            remnants.append((x, y, w, h))
            continue
        if px > x:
            remnants.append((x, y, px - x, h))
        if px + pw < x + w:
            remnants.append((px + pw, y, x + w - (px + pw), h))
        if py > y:
            remnants.append((x, y, w, py - y))
        if py + ph < y + h:
            remnants.append((x, py + ph, w, y + h - (py + ph)))
    return _prune(remnants)


def pack_layer(pallet_x, pallet_y, box_x, box_y):
    """Return placed rectangles as (x, y, width, height, rotated) in pallet coordinates."""
    free = [(0.0, 0.0, pallet_x, pallet_y)]
    placed = []
    while free:
        best = None
        for index, (x, y, w, h) in enumerate(free):
            for rotated, (bw, bh) in ((False, (box_x, box_y)), (True, (box_y, box_x))):
                if bw <= w + 1e-9 and bh <= h + 1e-9:
                    candidate = (y, x, int(rotated), index, rotated, bw, bh)
                    if best is None or candidate < best:
                        best = candidate
        if best is None:
            break
        _y, _x, _rot_flag, index, rotated, bw, bh = best
        x, y, _w, _h = free[index]
        placed.append((x, y, bw, bh, rotated))
        free = _split(free, x, y, bw, bh)
    return placed


def layer_count(box_z=BOX[2], max_height=PALLET_MAX_HEIGHT):
    return int((max_height + 1e-9) // box_z)


def all_slots():
    """Every slot of the pattern, ignoring whether the robot can reach it."""
    layer = pack_layer(PALLET_FOOTPRINT[0], PALLET_FOOTPRINT[1], BOX[0], BOX[1])
    slots = []
    for level in range(layer_count()):
        for x, y, width, height, rotated in layer:
            slots.append(
                Slot(
                    index=len(slots),
                    layer=level,
                    x=PALLET_ORIGIN[0] + x + width / 2.0,
                    y=PALLET_ORIGIN[1] + y + height / 2.0,
                    z=PALLET_TOP_Z + (level + 0.5) * BOX[2],
                    rotated=rotated,
                )
            )
    return slots


# ---------------------------------------------------------------------------
# Chassis pose for a slot
# ---------------------------------------------------------------------------

def _rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def _rect_overlap(ax, ay, ahw, ahh, rect):
    rx, ry, rw, rh = rect
    return abs(ax - (rx + rw / 2.0)) < ahw + rw / 2.0 and abs(ay - (ry + rh / 2.0)) < ahh + rh / 2.0


def _keep_out_rects():
    pallet = (PALLET_ORIGIN[0], PALLET_ORIGIN[1], PALLET_FOOTPRINT[0], PALLET_FOOTPRINT[1])
    stand = (
        PICKUP_XY[0] - STAND_SIZE[0] / 2.0,
        PICKUP_XY[1] - STAND_SIZE[1] / 2.0,
        STAND_SIZE[0],
        STAND_SIZE[1],
    )
    return pallet, stand


def shoulder_world(base):
    """World xy of the left-arm shoulder for a base pose (x, y, yaw)."""
    ox, oy = _rotate(SHOULDER_XY[0], SHOULDER_XY[1], base[2])
    return (base[0] + ox, base[1] + oy)


def reach_distance(base, slot):
    """Horizontal distance from the shoulder to the slot center."""
    sx, sy = shoulder_world(base)
    return math.hypot(slot.x - sx, slot.y - sy)


def base_is_clear(base):
    """True when the chassis footprint touches neither the pallet nor the stand."""
    c, s = abs(math.cos(base[2])), abs(math.sin(base[2]))
    half_x = c * BASE_HALF[0] + s * BASE_HALF[1]
    half_y = s * BASE_HALF[0] + c * BASE_HALF[1]
    half_x += BASE_MARGIN
    half_y += BASE_MARGIN
    return not any(_rect_overlap(base[0], base[1], half_x, half_y, rect) for rect in _keep_out_rects())


@lru_cache(maxsize=None)
def _base_search(slot_x, slot_y, reach, local_xs, local_ys, yaws):
    best = None
    for yaw_rank, yaw in enumerate(yaws):
        for local_x in local_xs:
            for local_y in local_ys:
                dx, dy = _rotate(local_x, local_y, yaw)
                base = (slot_x - dx, slot_y - dy, yaw)
                if not base_is_clear(base):
                    continue
                shoulder = shoulder_world(base)
                if math.hypot(slot_x - shoulder[0], slot_y - shoulder[1]) > reach:
                    continue
                cost = (
                    abs(local_x - LOCAL_X_PREFERRED)
                    + abs(local_y - LOCAL_Y_PREFERRED)
                    + (0.0 if yaw_rank == 0 else YAW_PENALTY)
                )
                key = (cost, yaw_rank, local_x, local_y)
                if best is None or key < best[0]:
                    best = (key, base)
    return None if best is None else best[1]


def base_for_slot(slot, reach=REACH_M, local_xs=LOCAL_X_OPTIONS, local_ys=LOCAL_Y_OPTIONS, yaws=BASE_YAWS):
    """Chassis pose (x, y, yaw) from which the left arm can set a box on this slot.

    Raises ``NoBasePose`` instead of returning a pose that cannot work.
    """
    base = _base_search(slot.x, slot.y, reach, tuple(local_xs), tuple(local_ys), tuple(yaws))
    if base is None:
        raise NoBasePose(
            f"slot {slot.index} (layer {slot.layer}, x={slot.x:.3f}, y={slot.y:.3f}) has no chassis pose "
            "that avoids the pallet and stand and stays within reach"
        )
    return base


def slot_is_reachable(slot, reach=REACH_M, **kwargs):
    try:
        base_for_slot(slot, reach=reach, **kwargs)
    except NoBasePose:
        return False
    return True


def pallet_slots(reach=REACH_M, **kwargs):
    """Slots the left arm can fill, in fill order.

    A slot is kept only when ``base_for_slot`` finds a chassis pose for it.
    Within a layer the far row (largest y) is filled first, so the arm never
    reaches over boxes it has just placed. Use ``unreachable_slots`` to see
    what was left out.
    """
    slots = [slot for slot in all_slots() if slot_is_reachable(slot, reach=reach, **kwargs)]
    slots.sort(key=lambda slot: (slot.layer, -slot.y, slot.x))
    return [Slot(index, slot.layer, slot.x, slot.y, slot.z, slot.rotated) for index, slot in enumerate(slots)]


def unreachable_slots(reach=REACH_M, **kwargs):
    return [slot for slot in all_slots() if not slot_is_reachable(slot, reach=reach, **kwargs)]


# ---------------------------------------------------------------------------
# Scene geometry and tool heights
# ---------------------------------------------------------------------------

def pallet_center():
    """Collision box for pallet plus riser, from the ground to PALLET_TOP_Z."""
    return (
        PALLET_ORIGIN[0] + PALLET_FOOTPRINT[0] / 2.0,
        PALLET_ORIGIN[1] + PALLET_FOOTPRINT[1] / 2.0,
        PALLET_TOP_Z / 2.0,
    )


def pallet_size():
    return (PALLET_FOOTPRINT[0], PALLET_FOOTPRINT[1], PALLET_TOP_Z)


def riser_center():
    """Center of the riser alone, resting on the pallet deck."""
    return (
        PALLET_ORIGIN[0] + PALLET_FOOTPRINT[0] / 2.0,
        PALLET_ORIGIN[1] + PALLET_FOOTPRINT[1] / 2.0,
        PALLET_THICKNESS + RISER_HEIGHT / 2.0,
    )


def riser_size():
    return (PALLET_FOOTPRINT[0], PALLET_FOOTPRINT[1], RISER_HEIGHT)


def stand_center():
    return (PICKUP_XY[0], PICKUP_XY[1], STAND_TOP_Z - STAND_SIZE[2] / 2.0)


def pickup_center():
    return (PICKUP_XY[0], PICKUP_XY[1], STAND_TOP_Z + BOX[2] / 2.0)


def spawn_center():
    """One millimetre above the settled pose so the new box drops onto the stand."""
    x, y, z = pickup_center()
    return (x, y, z + 0.001)


def pickup_box_top():
    return STAND_TOP_Z + BOX[2]


def tool_z_for_pick():
    """tool0 height that puts the gold lip on the box top."""
    return pickup_box_top() + LIP_FACE - CONTACT_PRESS


def carried_offset():
    """Distance from tool0 to the box center along tool +Z, which points down."""
    return BOX[2] / 2.0 + LIP_FACE - CONTACT_PRESS


def tool_z_for_place(slot):
    """tool0 height that puts the carried box bottom PLACE_CLEARANCE above the support.

    The box center hangs ``carried_offset()`` below tool0, so tool0 sits that far
    above the target center.
    """
    return slot.z + PLACE_CLEARANCE + carried_offset()


def place_too_low(slot):
    """True when the arm cannot get the tool down to this slot (see MIN_TOOL_Z)."""
    return tool_z_for_place(slot) < MIN_TOOL_Z - 1e-9


def required_deck_raise():
    """How far PALLET_TOP_Z must go up so that no slot is below MIN_TOOL_Z (0 if none is)."""
    lowest = min(tool_z_for_place(slot) for slot in all_slots())
    return max(0.0, MIN_TOOL_Z - lowest)


def lift_required(tool_z):
    """Unclamped lift that keeps the shoulder about 0.16 m above the suction tool."""
    return tool_z + 0.16 - SHOULDER_Z0


def lift_for_tool_z(tool_z, strict=False):
    """Clamped lift command. With ``strict`` an over-limit request raises instead of clamping."""
    lift = lift_required(tool_z)
    if strict and lift > LIFT_LIMIT + 1e-9:
        raise ValueError(f"tool z {tool_z:.3f} needs lift {lift:.3f}, limit is {LIFT_LIMIT}")
    if lift < 0.0:
        return 0.0
    if lift > LIFT_LIMIT:
        return LIFT_LIMIT
    return lift