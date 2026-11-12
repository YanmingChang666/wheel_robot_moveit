"""Point-to-plane scan matching for a 16-beam spinning lidar.

The robot is planar, so the alignment solves x, y and yaw. Each beam that
hits a wall or a pillar contributes a plane, which is the reason a 16-line
lidar holds the pose better than one horizontal ring.
"""

import math

import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def voxel_downsample(points, voxel):
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[index]


class VoxelMap:
    """Sparse 3D map. Each voxel keeps a handful of points for plane fits."""

    def __init__(self, voxel=0.25, limit=8):
        self.voxel = float(voxel)
        self.limit = int(limit)
        self.cells = {}

    def _key(self, point):
        scale = self.voxel
        return (
            int(math.floor(point[0] / scale)),
            int(math.floor(point[1] / scale)),
            int(math.floor(point[2] / scale)),
        )

    def insert(self, points):
        for point in points:
            key = self._key(point)
            bucket = self.cells.get(key)
            if bucket is None:
                self.cells[key] = [point]
            elif len(bucket) < self.limit:
                bucket.append(point)

    def plane(self, point, radius=0.55):
        """Return ``(normal, centroid)`` when the neighborhood is a plane."""
        base = self._key(point)
        gathered = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    bucket = self.cells.get((base[0] + dx, base[1] + dy, base[2] + dz))
                    if bucket:
                        gathered.extend(bucket)
        if len(gathered) < 6:
            return None
        samples = np.asarray(gathered, dtype=np.float64)
        delta = samples - point
        near = samples[np.einsum("ij,ij->i", delta, delta) < radius * radius]
        if len(near) < 6:
            return None
        center = near.mean(axis=0)
        centered = near - center
        cov = centered.T @ centered / len(near)
        values, vectors = np.linalg.eigh(cov)
        if values.sum() <= 1.0e-8 or values[0] / values.sum() > 0.08:
            return None
        normal = vectors[:, 0]
        return normal, center


def _solve3(ata, atb):
    matrix = np.array(ata, dtype=np.float64)
    matrix.flat[::4] += 1.0e-4
    try:
        return np.linalg.solve(matrix, np.asarray(atb, dtype=np.float64))
    except np.linalg.LinAlgError:
        return None


def align_scan(points, pose, voxel_map, iterations=8):
    """Align ``points`` (base frame, Nx3) to ``voxel_map``.

    ``pose`` is the prediction ``(x, y, yaw)`` of ``base_link`` in ``odom``.
    Returns ``(x, y, yaw, inliers)``. The input pose is returned unchanged when
    the scan does not lock onto the map.
    """
    if len(points) < 30 or not voxel_map.cells:
        return pose[0], pose[1], pose[2], 0
    sample = voxel_downsample(np.asarray(points, dtype=np.float64), 0.20)
    if len(sample) > 600:
        sample = sample[:: max(1, len(sample) // 600)]
    x, y, yaw = pose
    inliers = 0
    for _ in range(iterations):
        c = math.cos(yaw)
        s = math.sin(yaw)
        xb = sample[:, 0]
        yb = sample[:, 1]
        xo = c * xb - s * yb + x
        yo = s * xb + c * yb + y
        zo = sample[:, 2]
        ata = [[0.0, 0.0, 0.0] for _ in range(3)]
        atb = [0.0, 0.0, 0.0]
        inliers = 0
        for index in range(len(sample)):
            hit = voxel_map.plane((xo[index], yo[index], zo[index]))
            if hit is None:
                continue
            normal, center = hit
            residual = (
                normal[0] * (xo[index] - center[0])
                + normal[1] * (yo[index] - center[1])
                + normal[2] * (zo[index] - center[2])
            )
            if abs(residual) > 0.30:
                continue
            # d(R p)/d yaw = (-(yo - y), xo - x, 0)
            j2 = normal[0] * (-(yo[index] - y)) + normal[1] * (xo[index] - x)
            jac = (normal[0], normal[1], j2)
            for row in range(3):
                atb[row] += jac[row] * residual
                for col in range(3):
                    ata[row][col] += jac[row] * jac[col]
            inliers += 1
        if inliers < 40:
            return pose[0], pose[1], pose[2], inliers
        delta = _solve3(ata, atb)
        if delta is None:
            return pose[0], pose[1], pose[2], inliers
        # Residual is n·(p - q), so the step that reduces it is -delta.
        x -= max(-0.25, min(0.25, float(delta[0])))
        y -= max(-0.25, min(0.25, float(delta[1])))
        yaw = wrap(yaw - max(-0.15, min(0.15, float(delta[2]))))
    return x, y, yaw, inliers
