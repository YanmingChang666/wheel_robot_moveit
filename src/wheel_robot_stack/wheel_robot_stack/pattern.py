"""Layer pallet pattern for identical boxes.

Each layer is packed with Maximal Rectangles and the Bottom-Left rule from
Jukka Jylänki, "A Thousand Ways to Pack the Bin — A Practical Approach to
Two-Dimensional Rectangle Bin Packing" (2010):
http://pds25.egloos.com/pds/201504/21/98/RectangleBinPack.pdf

The same heuristic is what the MIT-licensed rectpack library implements. This
module is a small reimplementation so the stack node has no extra dependency.
Identical 351 x 287 mm faces on a 1200 x 1000 mm pallet produce a 3 x 3 grid.
Layers repeat until the pile would pass 1080 mm (10 layers, 1050 mm).
"""

from dataclasses import dataclass


BOX = (0.351, 0.287, 0.105)
PALLET_FOOTPRINT = (1.200, 1.000)
PALLET_MAX_HEIGHT = 1.080
PALLET_THICKNESS = 0.125
# Min corner of the wooden pallet measured from wheel_robot_suction_stack.usd.
# The prim order is scale then translate, and the translate is scaled, so the
# deck is x = -0.5985 .. 0.6015, y = 0.6214 .. 1.6214, z = 0 .. 0.125.
# Center (0.002, 1.121, 0.063). This is the box RViz draws.
PALLET_ORIGIN = (-0.5985, 0.6214)
# The pallet model sits on the ground, so the deck is its own height.
PALLET_TOP_Z = PALLET_THICKNESS
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
PLACE_CLEARANCE = 0.008
APPROACH_HEIGHT = 0.18
# Shoulder height at lift 0. The lift joint is commanded so the shoulder
# stays a little above the suction face.
SHOULDER_Z0 = 0.9105
LIFT_LIMIT = 0.7


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


def _contained(inner, outer):
    return (
        inner[0] >= outer[0] - 1e-9
        and inner[1] >= outer[1] - 1e-9
        and inner[0] + inner[2] <= outer[0] + outer[2] + 1e-9
        and inner[1] + inner[3] <= outer[1] + outer[3] + 1e-9
    )


def _prune(free):
    kept = []
    for index, rect in enumerate(free):
        if rect[2] < 1e-6 or rect[3] < 1e-6:
            continue
        if any(_contained(rect, other) for other_index, other in enumerate(free) if other_index != index):
            continue
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
    """Return placed rectangles as (x, y, width, height) in pallet coordinates."""
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


def pallet_slots(reach=REACH_M):
    """Slots the left arm is asked to fill, nearest row first within reach.

    Farther slots stay in the pattern result of ``all_slots`` but are left
    for a base move; this fixed arm cannot cover the whole 1200 x 1000 mm deck.
    """
    slots = []
    for slot in all_slots():
        dx = slot.x - SHOULDER_XY[0]
        dy = slot.y - SHOULDER_XY[1]
        if dx * dx + dy * dy <= reach * reach:
            slots.append(slot)
    slots.sort(key=lambda slot: (slot.layer, -slot.y, slot.x))
    return [Slot(index, slot.layer, slot.x, slot.y, slot.z, slot.rotated) for index, slot in enumerate(slots)]


def all_slots():
    layer = pack_layer(PALLET_FOOTPRINT[0], PALLET_FOOTPRINT[1], BOX[0], BOX[1])
    slots = []
    for level in range(layer_count()):
        for x, y, _w, _h, rotated in layer:
            slots.append(
                Slot(
                    index=len(slots),
                    layer=level,
                    x=PALLET_ORIGIN[0] + x + _w / 2.0,
                    y=PALLET_ORIGIN[1] + y + _h / 2.0,
                    z=PALLET_TOP_Z + (level + 0.5) * BOX[2],
                    rotated=rotated,
                )
            )
    return slots


def pallet_center():
    return (
        PALLET_ORIGIN[0] + PALLET_FOOTPRINT[0] / 2.0,
        PALLET_ORIGIN[1] + PALLET_FOOTPRINT[1] / 2.0,
        PALLET_TOP_Z - PALLET_THICKNESS / 2.0,
    )


def pallet_size():
    return (PALLET_FOOTPRINT[0], PALLET_FOOTPRINT[1], PALLET_THICKNESS)


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


def tool_z_for_place(slot):
    box_top = slot.support_z + BOX[2]
    return box_top + LIP_FACE + PLACE_CLEARANCE


def carried_offset():
    """Distance from tool0 to the box center along tool +Z, which points down."""
    return BOX[2] / 2.0 + LIP_FACE - CONTACT_PRESS


def _rect_overlap(ax, ay, ahw, ahh, rect):
    rx, ry, rw, rh = rect
    return abs(ax - (rx + rw / 2.0)) < ahw + rw / 2.0 and abs(ay - (ry + rh / 2.0)) < ahh + rh / 2.0


def base_for_slot(slot):
    """Park the chassis so the left arm can set the cup down on this slot.

    The search keeps the wheels off the pallet and the pickup stand. Offsets
    are in the base frame and were checked against a downward tool at deck height.
    """
    pallet = (PALLET_ORIGIN[0], PALLET_ORIGIN[1], PALLET_FOOTPRINT[0], PALLET_FOOTPRINT[1])
    stand = (
        PICKUP_XY[0] - STAND_SIZE[0] / 2.0,
        PICKUP_XY[1] - STAND_SIZE[1] / 2.0,
        STAND_SIZE[0],
        STAND_SIZE[1],
    )
    half = (0.26, 0.24)
    best = None
    for local_x in (0.05, 0.12, 0.20, 0.30, 0.38):
        for local_y in (0.32, 0.36, 0.40, 0.45):
            base_x = slot.x - local_x
            base_y = slot.y - local_y
            if _rect_overlap(base_x, base_y, half[0], half[1], pallet):
                continue
            if _rect_overlap(base_x, base_y, half[0], half[1], stand):
                continue
            cost = abs(local_x - 0.15) + abs(local_y - 0.42)
            if best is None or cost < best[0]:
                best = (cost, base_x, base_y)
    if best is None:
        return (slot.x - 0.15, PALLET_ORIGIN[1] - 0.55, 0.0)
    return (best[1], best[2], 0.0)


def lift_for_tool_z(tool_z):
    """Shoulder stays about 0.16 m above the suction tool."""
    lift = tool_z + 0.16 - SHOULDER_Z0
    if lift < 0.0:
        return 0.0
    if lift > LIFT_LIMIT:
        return LIFT_LIMIT
    return lift
