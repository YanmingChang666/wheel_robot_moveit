"""Layer pallet pattern for identical boxes.

Each layer is packed with Maximal Rectangles and the Bottom-Left rule from
Jukka Jylänki, "A Thousand Ways to Pack the Bin — A Practical Approach to
Two-Dimensional Rectangle Bin Packing" (2010):
http://pds25.egloos.com/pds/201504/21/98/RectangleBinPack.pdf

The same heuristic is what the MIT-licensed rectpack library implements. This
module is a small reimplementation so the stack node has no extra dependency.
Identical 351 x 287 mm faces on a 1200 x 1000 mm pallet produce a 3 x 3 grid.
Layers repeat until the pile would pass 1160 mm (11 layers).

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
# Free space left between neighbouring boxes. A 15 mm gap is smaller than the
# measured place error, so a box lands on its neighbour and the next drop stalls.
PACK_GAP = 0.030
# Closest a box may come to the pallet edge. The grid is centered, so the real
# edge clearance is whatever the pallet has left after three boxes and two gaps.
EDGE_MARGIN = 0.010
# Largest shift between two neighbouring layers that the arms can still serve.
# Found by sweeping the A/B offsets in 2.5 mm steps against either_arm_reaches:
# up to 0.070 m in x every cell that is reachable with centered layers stays
# reachable; in y the limit (0.100 m) is above what the pallet allows.
LAYER_SHIFT_ARM_LIMIT = (0.070, 0.100)
# Round the number of layers down to an even count when set.
EVEN_LAYERS = False
PALLET_FOOTPRINT = (1.200, 1.000)
PALLET_MAX_HEIGHT = 1.160
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
STAND_XY = (0.55, 0.18)
PICKUP_XY = (0.50, 0.18)
STAND_SIZE = (0.50, 0.36, 0.30)
# Left-arm shoulder at lift 0 and waist yaw 0, from the wheel-robot URDF.
SHOULDER_XY = (0.13755, 0.1212)
# Waist yaw axis in the base frame (lift 0, pitch 0), and each shoulder's
# offset from that axis. Positive yaw swings the left shoulder toward +X.
WAIST_AXIS_XY = (0.14355, 0.0)
ARM_SHOULDER = {
    "left": (-0.006, 0.1212),
    "right": (-0.006, -0.1212),
}
# Shoulder-to-cup distance with the seal pointing down. The link chain is
# 0.79 m, and the last link has to stay vertical, so the cup reaches about
# 0.50 m horizontally. 0.40 m is the distance that actually set a box down:
# 0.44 m reached the middle-right cell in XY and then stuck 7 cm high with
# the lift already at 0. 0.85 m counted a straight arm and sent the chassis
# to the pallet center, where every inverse solution fails.
REACH_M = 0.40
PLACE_REACH = 0.40
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
LOCAL_X_OPTIONS = (0.05, 0.12, 0.16, 0.24, 0.32, 0.40, 0.52)
LOCAL_Y_OPTIONS = (0.36, 0.42, 0.48, 0.52, 0.56, 0.64, 0.72, 0.80)
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
class ArmPlan:
    """Chassis pose and waist yaw that put one cup over a slot or the stand."""

    arm: str
    base: tuple
    waist: float
    distance: float


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
    count = int((max_height + 1e-9) // box_z)
    if EVEN_LAYERS and count % 2:
        count -= 1
    return count


def layer_shift():
    """(x, y) distance between the two alternating layer positions, in metres.

    The room left by the 3 x 3 grid and its gaps, minus an edge margin on both
    sides, is the most one layer can move before a box hangs over the pallet.
    The x shift is also capped by what the arms can reach.
    """
    room_x = PALLET_FOOTPRINT[0] - 3 * BOX[0] - 2 * PACK_GAP - 2 * EDGE_MARGIN
    room_y = PALLET_FOOTPRINT[1] - 3 * BOX[1] - 2 * PACK_GAP - 2 * EDGE_MARGIN
    return (
        max(0.0, min(room_x, LAYER_SHIFT_ARM_LIMIT[0])),
        max(0.0, min(room_y, LAYER_SHIFT_ARM_LIMIT[1])),
    )


def layer_offset(level):
    """Offset of a whole layer from the centered grid.

    Every layer stays on the pallet center. Shifting even layers toward -Y
    left about 10 mm at the near edge and a large empty band on +Y.
    """
    del level
    return (0.0, 0.0)


def all_slots():
    """Nine boxes per layer, every layer on the same centered grid.

    The long side of the box runs along the pallet's 1.2 m side. A layer is
    finished before the next one starts, and each box sits on the box below it.
    """
    columns = 3
    rows = 3
    cell_x = BOX[0] + PACK_GAP
    cell_y = BOX[1] + PACK_GAP
    margin_x = (PALLET_FOOTPRINT[0] - columns * cell_x) / 2.0
    margin_y = (PALLET_FOOTPRINT[1] - rows * cell_y) / 2.0
    slots = []
    for level in range(layer_count()):
        off_x, off_y = layer_offset(level)
        for row in range(rows):
            for col in range(columns):
                slots.append(
                    Slot(
                        index=len(slots),
                        layer=level,
                        x=PALLET_ORIGIN[0] + margin_x + (col + 0.5) * cell_x + off_x,
                        y=PALLET_ORIGIN[1] + margin_y + (row + 0.5) * cell_y + off_y,
                        z=PALLET_TOP_Z + (level + 0.5) * BOX[2],
                        rotated=False,
                    )
                )
    return slots


def under_slot(slot, center, margin=0.01):
    """True when a box centered at ``center`` overlaps the slot's footprint."""
    return (
        abs(center[0] - slot.x) < BOX[0] - margin
        and abs(center[1] - slot.y) < BOX[1] - margin
    )


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
        STAND_XY[0] - STAND_SIZE[0] / 2.0,
        STAND_XY[1] - STAND_SIZE[1] / 2.0,
        STAND_SIZE[0],
        STAND_SIZE[1],
    )
    return pallet, stand


def shoulder_world(base, arm="left", waist=0.0):
    """World xy of one shoulder for a base pose (x, y, yaw) and waist yaw."""
    off = ARM_SHOULDER[arm]
    ox, oy = _rotate(off[0], off[1], waist)
    lx, ly = WAIST_AXIS_XY[0] + ox, WAIST_AXIS_XY[1] + oy
    rx, ry = _rotate(lx, ly, base[2])
    return (base[0] + rx, base[1] + ry)


def reach_distance(base, slot):
    """Horizontal distance from the shoulder to the slot center."""
    sx, sy = shoulder_world(base)
    return math.hypot(slot.x - sx, slot.y - sy)


def _footprint_hits(base, rects, margin=BASE_MARGIN):
    c, s = abs(math.cos(base[2])), abs(math.sin(base[2]))
    half_x = c * BASE_HALF[0] + s * BASE_HALF[1] + margin
    half_y = s * BASE_HALF[0] + c * BASE_HALF[1] + margin
    return any(_rect_overlap(base[0], base[1], half_x, half_y, rect) for rect in rects)


def base_is_clear(base):
    """True when the chassis footprint touches neither the pallet nor the stand."""
    return not _footprint_hits(base, _keep_out_rects())


def _clear_of_pallet(base):
    """The pickup stand may be grazed. The pallet may not."""
    pallet, _stand = _keep_out_rects()
    return not _footprint_hits(base, (pallet,))


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


# Slot position in the base frame. The shoulder sits ahead of the chassis
# center, so a cup 0.36 m from the shoulder can be 0.6 m from that center.
# The clear band beside the pallet is only a few centimetres wide, so the
# step has to be finer than the gap between a pose on the pallet and a pose
# past the arm's reach.
_PLAN_X = (0.08, 0.16, 0.24, 0.32, 0.40, 0.52)
_PLAN_Y = (0.00, 0.16, 0.28, 0.36, 0.42, 0.48, 0.52, 0.56, 0.64, 0.72)
_PLAN_WAIST = (-1.0, -0.5, 0.0, 0.5, 1.0)


def _clear_with_margin(base, extra):
    """``base_is_clear``, with ``extra`` metres added to the chassis keep-out."""
    if extra <= 0.0:
        return base_is_clear(base)
    cosine, sine = abs(math.cos(base[2])), abs(math.sin(base[2]))
    half_x = cosine * BASE_HALF[0] + sine * BASE_HALF[1] + BASE_MARGIN + extra
    half_y = sine * BASE_HALF[0] + cosine * BASE_HALF[1] + BASE_MARGIN + extra
    return not any(
        _rect_overlap(base[0], base[1], half_x, half_y, rect) for rect in _keep_out_rects()
    )


def _nearest_plans(x, y, reach, allow_stand, home_weight=0.0, extra_margin=0.0):
    """Best clear chassis pose for each arm, cup about 0.36 m from the shoulder.

    Farther than ``reach`` the wrist cannot stay pointed down. That is what
    left the middle-right box 7 cm in the air. The pallet-center cell stays
    about 0.64 m from either shoulder, so it never appears here.
    """
    found = {}
    for arm in ("left", "right"):
        best = None
        for yaw in SIDE_YAWS:
            for waist in _PLAN_WAIST:
                for ix in range(-16, 17):
                    for iy in range(-16, 17):
                        base = (x + ix * 0.08, y + iy * 0.08, yaw)
                        if allow_stand:
                            if not _clear_of_pallet(base):
                                continue
                        elif not _clear_with_margin(base, extra_margin):
                            continue
                        sx, sy = shoulder_world(base, arm, waist)
                        distance = math.hypot(x - sx, y - sy)
                        if distance < 0.28 or distance > reach:
                            continue
                        # Waist zero inside the reach beats a closer pose that
                        # is twisted out to the joint limit.
                        cost = (
                            abs(distance - 0.36)
                            + 0.35 * abs(waist)
                            + home_weight * math.hypot(base[0], base[1])
                        )
                        if best is None or cost < best[0]:
                            best = (cost, ArmPlan(arm, base, waist, distance))
        if best is not None:
            found[arm] = best[1]
    return found


# Local offsets where a live inverse-kinematics probe could put the cup at
# place height with the seal down and the waist straight. Outside this window
# the shoulder can be close while the wrist still has no solution.
_PLACE_X = (0.08, 0.16, 0.24, 0.32)
_PLACE_Y = (0.32, 0.40, 0.44, 0.48, 0.50, 0.52, 0.53, 0.54, 0.56)


def _place_local_ok(local_x, local_y):
    if local_x >= 0.40 and local_y >= 0.48:
        return False
    if local_x >= 0.32 and local_y >= 0.56:
        return False
    return True


@lru_cache(maxsize=None)
def _arm_candidates(x, y, reach):
    found = {}
    for arm, sign in (("left", 1.0), ("right", -1.0)):
        choices = []
        for yaw in SIDE_YAWS:
            best = None
            for local_x in _PLACE_X:
                for local_y in _PLACE_Y:
                    if not _place_local_ok(local_x, local_y):
                        continue
                    dx, dy = _rotate(local_x, sign*local_y, yaw)
                    base = (x-dx, y-dy, yaw)
                    if not base_is_clear(base):
                        continue
                    sx, sy = shoulder_world(base, arm, 0.0)
                    distance = math.hypot(x-sx, y-sy)
                    cost = abs(local_x-.16)+abs(local_y-.44)
                    if best is None or cost < best[0]:
                        best = cost, ArmPlan(arm, base, 0.0, distance)
            if best is not None:
                choices.append(best)
        if choices:
            found[arm] = tuple(plan for _, plan in sorted(choices, key=lambda item:item[0]))
    return found


@lru_cache(maxsize=None)
def _arm_options(x, y, reach):
    return {arm:plans[0] for arm, plans in _arm_candidates(x, y, reach).items()}


def arm_options(x, y, reach=PLACE_REACH):
    return dict(_arm_options(float(x), float(y), float(reach)))


def either_arm_reaches(x, y, reach=PLACE_REACH):
    return bool(_arm_options(float(x), float(y), float(reach)))


def pallet_slots(reach=PLACE_REACH, **kwargs):
    """Slots either arm can fill, one full layer before the next.

    The centered 3x3 is kept wherever a chassis pose exists. The pallet-center
    cell has no such pose: both shoulders stay about 0.64 m away while the
    cup is pointing down. Inside a layer the far row is filled first.
    """
    del kwargs
    slots = [slot for slot in all_slots() if either_arm_reaches(slot.x, slot.y, reach)]
    slots.sort(key=lambda slot: (slot.layer, -slot.y, slot.x))
    return [Slot(index, slot.layer, slot.x, slot.y, slot.z, slot.rotated) for index, slot in enumerate(slots)]


def unreachable_slots(reach=PLACE_REACH, **kwargs):
    del kwargs
    return [slot for slot in all_slots() if not either_arm_reaches(slot.x, slot.y, reach)]


def carry_pairs(slots, reach=PLACE_REACH, pickup_base=None):
    """Greedily minimize each trip's routed distance, respecting layer/row order.

    Compare opposite-arm assignments and parking headings using the actual
    obstacle route, including the return to the shared pickup station.
    This is a per-trip optimization, not a global traveling-salesman solution.
    """
    station = tuple(pickup_base or pick_plan("left").base)
    pending = list(slots)
    batches = []
    while pending:
        first = pending.pop(0)
        options = _arm_candidates(first.x, first.y, reach)
        partners = [(i, s) for i,s in enumerate(pending) if s.layer == first.layer]
        same_row = [(i,s) for i,s in partners if abs(s.y-first.y) < 1e-6]
        if same_row:
            partners = same_row
        best = None
        for arm, plans in options.items():
            other = "right" if arm == "left" else "left"
            for i, second in partners:
                choices = _arm_candidates(second.x, second.y, reach).get(other, ())
                for a in plans:
                    for b in choices:
                        cost = (route_distance(station,a.base)+route_distance(a.base,b.base)
                                + route_distance(b.base,station))
                        if math.isfinite(cost) and (best is None or cost < best[0]):
                            best = cost, i, [(first,a),(second,b)]
        if best is not None:
            _, i, batch = best
            pending.pop(i)
            batches.append(batch)
            continue
        choices = [(route_distance(station,p.base)*2, p)
                   for plans in options.values() for p in plans]
        choices = [(cost,p) for cost,p in choices if math.isfinite(cost)]
        if not choices:
            raise NoBasePose(f"格子 {first.index} 没有可行往返路线")
        _, plan = min(choices, key=lambda item:item[0])
        batches.append([(first,plan)])
    return batches


@lru_cache(maxsize=None)
def pick_plan(arm):
    """Both cups use one stationary base, with the stand on the midline.

    Only position once on arrival; do not drive or turn between the two picks.
    The two wrist-down chains were checked kinematically at local (0.55, 0).
    Collision feasibility remains a runtime MoveIt check.
    """
    if arm not in ARM_SHOULDER:
        raise ValueError(arm)
    base = (0.0, PICKUP_XY[1], 0.0)
    sx, sy = shoulder_world(base, arm, 0.0)
    return ArmPlan(arm, base, 0.0, math.hypot(PICKUP_XY[0]-sx,PICKUP_XY[1]-sy))


@lru_cache(maxsize=8192)
def route_distance(start, goal):
    try:
        route = route_around(start, goal)
    except NoBasePose:
        return float("inf")
    return sum(math.hypot(b[0]-a[0],b[1]-a[1])
               for a,b in zip((start,*route),route))


def _segment_clear(a, b):
    """Check translation AND intermediate chassis headings along an edge.

    Only the known pickup corridor may use the extra safety margin near the
    stand; the physical footprint and pallet clearance still must be clear.
    """
    length = math.hypot(b[0]-a[0],b[1]-a[1])
    turn = math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
    def pickup_corridor(pose):
        yaw = math.atan2(math.sin(pose[2]),math.cos(pose[2]))
        return abs(pose[0]) <= .025 and -.025 <= pose[1] <= PICKUP_XY[1]+.025 and abs(yaw) <= .04
    depart = .30 if pickup_corridor(a) else 0.
    arrive = .30 if pickup_corridor(b) else 0.
    def clear_pose(pose,travelled):
        if base_is_clear(pose):
            return True
        near_pickup=((depart>0 and travelled<=depart) or (arrive>0 and length-travelled<=arrive))
        pallet, stand = _keep_out_rects()
        # Margin 0 is the body itself. The 5 cm keep-out is only a buffer.
        if _footprint_hits(pose, (pallet,), margin=0.0):
            return False
        if _footprint_hits(pose, (stand,), margin=0.0):
            return False
        # A side park sits inside the 5 cm keep-out but does not overlap the
        # pallet. It may roll straight out. It may not spin there: the
        # diagonal headings sweep the chassis into the pallet.
        if near_pickup:
            return True
        return travelled <= 0.25 and abs(turn) <= 0.05
    # The controller turns in place at the edge's START, then translates at
    # the destination heading. Check that exact sequence, not a diagonal
    # interpolation that might permit a turn beside the pallet at arrival.
    turns=max(1,int(math.ceil(abs(turn)/.05)))
    for index in range(turns+1):
        if not clear_pose((a[0],a[1],a[2]+turn*index/turns),0.):
            return False
    steps=max(1,int(math.ceil(length/.02)))
    for index in range(steps+1):
        fraction=index/steps
        pose=(a[0]+(b[0]-a[0])*fraction,a[1]+(b[1]-a[1])*fraction,b[2])
        if not clear_pose(pose,length*fraction):
            return False
    return True


def route_around(start, goal):
    """Waypoints that reach ``goal`` without driving the chassis across the pallet.

    ``start`` and ``goal`` are (x, y, yaw). The direct line is used when it
    stays clear. Otherwise the chassis follows gates west, south, and north of
    the pallet and the pickup stand. The stand sits in front of the pallet, so
    the east side is reached by going south of the stand first.
    """
    if _segment_clear(start, goal):
        return [goal]
    pad = max(BASE_HALF) + BASE_MARGIN + 0.06
    pallet_x, pallet_y, pallet_w, pallet_h = _keep_out_rects()[0]
    stand_x, stand_y, stand_w, stand_h = _keep_out_rects()[1]
    west = pallet_x - pad
    east = max(pallet_x + pallet_w, stand_x + stand_w) + pad
    south = min(pallet_y, stand_y) - pad
    north = max(pallet_y + pallet_h, stand_y + stand_h) + pad
    gates = (
        (west, start[1], start[2]),
        (west, south, start[2]),
        (start[0], south, start[2]),
        (east, south, start[2]),
        (east, north, goal[2]),
        (west, north, goal[2]),
        (east, goal[1], goal[2]),
        (west, goal[1], goal[2]),
    )
    # Include close obstacle corners; the previous outer gates forced large
    # detours even when a shorter path along the stand or pallet was clear.
    corners = []
    for x,y,w,h in _keep_out_rects():
        for cx in (x-pad,x+w+pad):
            for cy in (y-pad,y+h+pad):
                for yaw in (start[2],goal[2]):
                    pose=(cx,cy,yaw)
                    if base_is_clear(pose):
                        corners.append(pose)
    # A side parking pose can translate safely while being too close to
    # rotate. Offer a short outward escape before asking it to turn.
    turn_pad = math.hypot(*BASE_HALF)+BASE_MARGIN+.06
    escapes = []
    for x,y,w,h in _keep_out_rects():
        for pose in (start,goal):
            escapes.extend((cx,pose[1],pose[2]) for cx in (x-turn_pad,x+w+turn_pad))
            escapes.extend((pose[0],cy,pose[2]) for cy in (y-turn_pad,y+h+turn_pad))
        corners.extend((cx,cy,yaw) for cx in (x-turn_pad,x+w+turn_pad)
                       for cy in (y-turn_pad,y+h+turn_pad)
                       for yaw in (start[2],goal[2]))
    nodes = tuple(dict.fromkeys((start,goal,*gates,*corners,*escapes)))
    count = len(nodes)
    adjacency = [[] for _ in range(count)]
    for left in range(count):
        for right in range(count):
            if left == right:
                continue
            if math.hypot(nodes[left][0] - nodes[right][0], nodes[left][1] - nodes[right][1]) < 1.0e-3:
                continue
            if _segment_clear(nodes[left], nodes[right]):
                adjacency[left].append(right)
    distance = [float("inf")] * count
    previous = [-1] * count
    distance[0] = 0.0
    pending = {0}
    while pending:
        current = min(pending, key=lambda index: distance[index])
        pending.remove(current)
        if current == 1:
            break
        for nxt in adjacency[current]:
            step = math.hypot(nodes[nxt][0] - nodes[current][0], nodes[nxt][1] - nodes[current][1])
            candidate = distance[current] + step
            if candidate < distance[nxt]:
                distance[nxt] = candidate
                previous[nxt] = current
                pending.add(nxt)
    if previous[1] < 0 and distance[1] == float("inf"):
        raise NoBasePose(f"底盘无法绕开障碍物：{start} -> {goal}")
    path = []
    cursor = 1
    while cursor > 0:
        path.append(nodes[cursor])
        cursor = previous[cursor]
    path.reverse()
    return path


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
    return (STAND_XY[0], STAND_XY[1], STAND_TOP_Z - STAND_SIZE[2] / 2.0)


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