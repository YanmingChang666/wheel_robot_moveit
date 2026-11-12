"""Planar error-state filter for wheel twist and IMU.

State is ``[x, y, yaw, vx, vy, wz]`` in the ``odom`` / ``base_link`` frames.
Wheel odometry supplies the body velocity. The IMU gyro and a yaw taken
relative to the first orientation correct the heading the wheels accumulate
when a tire slips.
"""

import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(x, y, z, w):
    """Yaw of a ROS ``geometry_msgs/Quaternion`` (x, y, z, w)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def relative_yaw(q, q0):
    """Yaw of ``q0^-1 * q``. Both quaternions are ``(x, y, z, w)``."""
    x0, y0, z0, w0 = q0
    x, y, z, w = q
    ix, iy, iz, iw = -x0, -y0, -z0, w0
    rw = iw * w - ix * x - iy * y - iz * z
    rx = iw * x + ix * w + iy * z - iz * y
    ry = iw * y - ix * z + iy * w + iz * x
    rz = iw * z + ix * y - iy * x + iz * w
    return quat_to_yaw(rx, ry, rz, rw)


def quat_valid(q):
    return sum(value * value for value in q) > 0.5


def _zeros(rows, cols):
    return [[0.0] * cols for _ in range(rows)]


def _eye(n):
    out = _zeros(n, n)
    for i in range(n):
        out[i][i] = 1.0
    return out


def _mul(a, b):
    out = _zeros(len(a), len(b[0]))
    for i, row in enumerate(a):
        for k, aik in enumerate(row):
            if aik == 0.0:
                continue
            brow = b[k]
            out_row = out[i]
            for j, bkj in enumerate(brow):
                out_row[j] += aik * bkj
    return out


def _transpose(a):
    return [list(col) for col in zip(*a)]


def _add(a, b):
    return [[a[i][j] + b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def _sub(a, b):
    return [[a[i][j] - b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def _sym(a):
    out = [row[:] for row in a]
    for i in range(len(out)):
        for j in range(i):
            value = 0.5 * (out[i][j] + out[j][i])
            out[i][j] = value
            out[j][i] = value
        if out[i][i] < 1.0e-9:
            out[i][i] = 1.0e-9
    return out


def _invert(matrix):
    """Gauss-Jordan inverse. ``matrix`` is square and small (1x1 or 3x3)."""
    n = len(matrix)
    aug = [row[:] + eye_row for row, eye_row in zip(matrix, _eye(n))]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(aug[row][col]))
        if abs(aug[pivot][col]) < 1.0e-12:
            raise ValueError("singular covariance")
        aug[col], aug[pivot] = aug[pivot], aug[col]
        scale = aug[col][col]
        aug[col] = [value / scale for value in aug[col]]
        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            aug[row] = [aug[row][j] - factor * aug[col][j] for j in range(2 * n)]
    return [row[n:] for row in aug]


def _diag(values):
    out = _zeros(len(values), len(values))
    for i, value in enumerate(values):
        out[i][i] = value
    return out


class PlanarEkf:
    """Constant-velocity planar EKF. Distances are metres and yaw is radians."""

    def __init__(self):
        self.x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.P = _diag([1.0e-4, 1.0e-4, 1.0e-4, 0.05, 0.05, 0.05])

    def predict(self, dt):
        if dt <= 0.0 or dt > 0.5:
            return
        x, y, yaw, vx, vy, wz = self.x
        c = math.cos(yaw)
        s = math.sin(yaw)
        self.x[0] = x + (vx * c - vy * s) * dt
        self.x[1] = y + (vx * s + vy * c) * dt
        self.x[2] = wrap(yaw + wz * dt)
        F = _eye(6)
        F[0][2] = (-vx * s - vy * c) * dt
        F[0][3] = c * dt
        F[0][4] = -s * dt
        F[1][2] = (vx * c - vy * s) * dt
        F[1][3] = s * dt
        F[1][4] = c * dt
        F[2][5] = dt
        q = _diag(
            [
                (0.02 * dt) ** 2,
                (0.02 * dt) ** 2,
                (0.03 * dt) ** 2,
                (0.5 * dt) ** 2,
                (0.5 * dt) ** 2,
                (0.6 * dt) ** 2,
            ]
        )
        self.P = _sym(_add(_mul(_mul(F, self.P), _transpose(F)), q))

    def update(self, z, H, R, wrap_rows=()):
        """Linear update. ``wrap_rows`` are measurement rows whose innovation is an angle."""
        hx = _mul(H, [[value] for value in self.x])
        y = []
        for i, measured in enumerate(z):
            innovation = measured - hx[i][0]
            if i in wrap_rows:
                innovation = wrap(innovation)
                if abs(innovation) > 0.8:
                    return False
            y.append([innovation])
        S = _add(_mul(_mul(H, self.P), _transpose(H)), R)
        try:
            S_inv = _invert(S)
        except ValueError:
            return False
        K = _mul(_mul(self.P, _transpose(H)), S_inv)
        dx = _mul(K, y)
        for i in range(6):
            self.x[i] += dx[i][0]
        self.x[2] = wrap(self.x[2])
        I_KH = _sub(_eye(6), _mul(K, H))
        self.P = _sym(_mul(I_KH, self.P))
        return True

    def update_twist(self, vx, vy, wz):
        # Wheel yaw rate is down-weighted. The gyro measurement below is tighter,
        # so a skidding tire does not pull the heading as hard as the IMU.
        H = [
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ]
        return self.update([vx, vy, wz], H, _diag([0.02, 0.02, 0.08]))

    def update_gyro(self, wz):
        H = [[0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]
        return self.update([wz], H, _diag([0.004]))

    def update_yaw(self, yaw):
        H = [[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]
        return self.update([yaw], H, _diag([0.01]), wrap_rows=(0,))
