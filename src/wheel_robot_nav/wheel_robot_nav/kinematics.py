"""Four-wheel independent steering kinematics for wheel_robot.

Steering joints yaw about +Z. At steer angle 0 the tire rolls along base_link +X.
Wheels 1 and 4 have joint axis -Y, so a positive joint velocity drives the robot
in -X. Wheels 2 and 3 have joint axis +Y and drive in +X. ``drive_sign`` folds
that into one rolling-speed convention.
"""

import math

WHEEL_RADIUS = 0.070
STEER_LIMIT = 2.967

# steer joint, drive joint, x, y, drive_sign
MODULES = (
    ("wheel_turn_1_Link", "wheel_run_1_Link", 0.23, 0.218, -1.0),
    ("wheel_turn_2_Link", "wheel_run_2_Link", 0.23, -0.218, 1.0),
    ("wheel_turn_3_Link", "wheel_run_3_Link", -0.23, -0.218, 1.0),
    ("wheel_turn_4_Link", "wheel_run_4_Link", -0.23, 0.218, -1.0),
)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _closest_steer(desired, current):
    """Pick the joint angle within limits that points the tire along ``desired``.

    Heading and heading + pi are the same rolling direction if the drive speed
    is reversed. The closer legal angle is used so a wheel does not spin the
    long way around.
    """
    options = []
    for turn, direction in ((0.0, 1.0), (math.pi, -1.0)):
        angle = _wrap(desired + turn)
        if abs(angle) <= STEER_LIMIT:
            options.append((abs(_wrap(angle - current)), angle, direction))
    if not options:
        return max(-STEER_LIMIT, min(STEER_LIMIT, _wrap(desired))), 1.0
    _, angle, direction = min(options)
    return angle, direction


def steer_error(measured, target):
    return abs(_wrap(measured - target))


def twist_to_wheels(vx, vy, wz, hold_angles):
    """Return steer angles (rad) and drive joint velocities (rad/s).

    ``hold_angles`` is used when a module's commanded speed is near zero, so the
    steering motors do not twitch.
    """
    angles = []
    drives = []
    for index, (_, _, x, y, sign) in enumerate(MODULES):
        vix = vx - wz * y
        viy = vy + wz * x
        speed = math.hypot(vix, viy)
        if speed < 1.0e-3:
            angle = hold_angles[index]
            speed = 0.0
        else:
            angle, direction = _closest_steer(math.atan2(viy, vix), hold_angles[index])
            speed *= direction
        angles.append(angle)
        drives.append(sign * speed / WHEEL_RADIUS)
    return angles, drives


def wheels_to_twist(steer, drive):
    """Least-squares body twist (vx, vy, wz) from four measured modules."""
    rows = []
    values = []
    for index, (_, _, x, y, sign) in enumerate(MODULES):
        if steer[index] is None or drive[index] is None:
            continue
        speed = sign * drive[index] * WHEEL_RADIUS
        vix = speed * math.cos(steer[index])
        viy = speed * math.sin(steer[index])
        rows.append((1.0, 0.0, -y, vix))
        rows.append((0.0, 1.0, x, viy))
    if len(rows) < 3:
        return 0.0, 0.0, 0.0
    # Solve A [vx, vy, wz]^T = b with the normal equations.
    ata = [[0.0, 0.0, 0.0] for _ in range(3)]
    atb = [0.0, 0.0, 0.0]
    for a0, a1, a2, b in rows:
        coeffs = (a0, a1, a2)
        for r in range(3):
            atb[r] += coeffs[r] * b
            for c in range(3):
                ata[r][c] += coeffs[r] * coeffs[c]
    return _solve3(ata, atb)


def _solve3(a, b):
    """Cramer's rule for a 3x3 system. Returns zeros if the matrix is singular."""
    def det(m):
        return (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )

    base = det(a)
    if abs(base) < 1.0e-9:
        return 0.0, 0.0, 0.0
    cols = []
    for col in range(3):
        replaced = [list(row) for row in a]
        for r in range(3):
            replaced[r][col] = b[r]
        cols.append(det(replaced) / base)
    return cols[0], cols[1], cols[2]
