"""16-line lidar localization and occupancy mapping.

Wheel speed and the IMU gyro predict the planar motion. A full scan from the
head-mounted 16-beam lidar then corrects x, y and yaw against a voxel map
(point-to-plane). The same returns, between 0.25 m and 1.7 m above the ground,
fill the occupancy grid Nav2 plans on.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from wheel_robot_nav.lidar_match import VoxelMap, align_scan, voxel_downsample, wrap

# lidar_link on waist_yaw_Link at zero joints: lift origin 0.58, waist 0.081, mount 0.66.
_FALLBACK_LIDAR = np.array([0.14355, 0.0, 1.321])


def _cloud_xyz(msg):
    fields = {field.name: field for field in msg.fields}
    if any(name not in fields for name in ("x", "y", "z")):
        return None
    count = int(msg.width) * int(msg.height)
    step = int(msg.point_step)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < count * step or step < 12:
        return None
    raw = raw[: count * step].reshape(count, step)

    def column(name):
        offset = fields[name].offset
        blob = np.ascontiguousarray(raw[:, offset : offset + 4])
        return blob.view(np.float32).reshape(-1)

    points = np.stack([column("x"), column("y"), column("z")], axis=1).astype(np.float64)
    finite = np.isfinite(points).all(axis=1)
    return points[finite]


def _quat_to_matrix(x, y, z, w):
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


class Occupancy:
    def __init__(self):
        self.resolution = 0.05
        self.origin = -15.0
        self.size = 600
        self.data = np.full(self.size * self.size, -1, dtype=np.int8)

    def _index(self, x, y):
        col = int((x - self.origin) / self.resolution)
        row = int((y - self.origin) / self.resolution)
        if col < 0 or row < 0 or col >= self.size or row >= self.size:
            return None
        return row * self.size + col

    def _mark_free(self, x0, y0, x1, y1):
        start = self._index(x0, y0)
        end = self._index(x1, y1)
        if start is None or end is None:
            return
        c0, r0 = start % self.size, start // self.size
        c1, r1 = end % self.size, end // self.size
        steps = max(abs(c1 - c0), abs(r1 - r0))
        if steps == 0:
            return
        for step in range(steps):
            col = c0 + (c1 - c0) * step // steps
            row = r0 + (r1 - r0) * step // steps
            index = row * self.size + col
            if self.data[index] < 0:
                self.data[index] = 0

    def integrate(self, origin_xy, hits):
        for x, y in hits:
            self._mark_free(origin_xy[0], origin_xy[1], x, y)
            index = self._index(x, y)
            if index is not None:
                self.data[index] = 100


class LidarLocalization(Node):
    def __init__(self):
        super().__init__("lidar_localization")
        self.declare_parameter("min_z", 0.25)
        self.declare_parameter("max_z", 1.70)
        self.declare_parameter("min_range", 0.45)
        sensor = QoSProfile(depth=5)
        reliable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._imu_wz = None
        self._imu_stamp = None
        self._last = None
        self._mapped = False
        self._last_map = None
        self._voxel = VoxelMap()
        self._grid = Occupancy()
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._tf = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "/odometry/filtered", sensor)
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", reliable)
        self.create_subscription(Odometry, "/odom", self._on_odom, sensor)
        self.create_subscription(Imu, "/imu", self._on_imu, sensor)
        self.create_subscription(PointCloud2, "/points", self._on_cloud, sensor)
        self.create_timer(0.05, self._on_timer)

    def _seconds(self, stamp):
        return rclpy.time.Time.from_msg(stamp).nanoseconds * 1.0e-9

    def _propagate(self, stamp):
        if self._last is None:
            self._last = stamp
            return
        dt = stamp - self._last
        if dt <= 0.0 or dt > 0.5:
            if dt > 0.5:
                self._last = stamp
            return
        wz = self._wz
        if self._imu_stamp is not None and stamp - self._imu_stamp < 0.2 and self._imu_wz is not None:
            wz = self._imu_wz
        c = math.cos(self._yaw)
        s = math.sin(self._yaw)
        self._x += (self._vx * c - self._vy * s) * dt
        self._y += (self._vx * s + self._vy * c) * dt
        self._yaw = wrap(self._yaw + wz * dt)
        self._last = stamp

    def _on_odom(self, msg):
        self._vx = msg.twist.twist.linear.x
        self._vy = msg.twist.twist.linear.y
        self._wz = msg.twist.twist.angular.z

    def _on_imu(self, msg):
        self._imu_wz = msg.angular_velocity.z
        self._imu_stamp = self._seconds(msg.header.stamp)

    def _lidar_to_base(self):
        try:
            tf = self._buffer.lookup_transform("base_link", "lidar_link", rclpy.time.Time())
        except Exception:
            return np.eye(3), _FALLBACK_LIDAR.copy()
        t = tf.transform.translation
        q = tf.transform.rotation
        rotation = _quat_to_matrix(q.x, q.y, q.z, q.w)
        return rotation, np.array([t.x, t.y, t.z])

    def _on_cloud(self, msg):
        points = _cloud_xyz(msg)
        if points is None or len(points) < 50:
            return
        ranges = np.linalg.norm(points, axis=1)
        points = points[ranges > float(self.get_parameter("min_range").value)]
        if len(points) < 50:
            return
        rotation, translation = self._lidar_to_base()
        base = points @ rotation.T + translation
        min_z = float(self.get_parameter("min_z").value)
        max_z = float(self.get_parameter("max_z").value)
        band = base[(base[:, 2] > min_z) & (base[:, 2] < max_z)]
        if len(band) < 50:
            return
        stamp = self._seconds(msg.header.stamp)
        self._propagate(stamp)
        pose = (self._x, self._y, self._yaw)
        if self._mapped:
            x, y, yaw, inliers = align_scan(band, pose, self._voxel)
            if inliers >= 40:
                self._x, self._y, self._yaw = x, y, yaw
        c = math.cos(self._yaw)
        s = math.sin(self._yaw)
        world_xy = np.column_stack(
            (
                c * band[:, 0] - s * band[:, 1] + self._x,
                s * band[:, 0] + c * band[:, 1] + self._y,
                band[:, 2],
            )
        )
        moved = self._last_map is None or math.hypot(self._x - self._last_map[0], self._y - self._last_map[1]) > 0.30
        turned = self._last_map is None or abs(wrap(self._yaw - self._last_map[2])) > 0.20
        if moved or turned:
            self._voxel.insert(voxel_downsample(world_xy, 0.20))
            self._last_map = (self._x, self._y, self._yaw)
        # A few hundred rays are enough to open the free space in front of the robot.
        step = max(1, len(world_xy) // 400)
        self._grid.integrate((self._x, self._y), world_xy[::step, :2])
        self._mapped = True
        self._publish(msg.header.stamp, force_map=True)

    def _on_timer(self):
        stamp = self.get_clock().now()
        self._propagate(stamp.nanoseconds * 1.0e-9)
        self._publish(stamp.to_msg(), force_map=False)

    def _publish(self, stamp, force_map):
        half = 0.5 * self._yaw
        qz = math.sin(half)
        qw = math.cos(half)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._wz
        self._odom_pub.publish(odom)

        map_tf = TransformStamped()
        map_tf.header.stamp = stamp
        map_tf.header.frame_id = "map"
        map_tf.child_frame_id = "odom"
        map_tf.transform.rotation.w = 1.0
        base_tf = TransformStamped()
        base_tf.header.stamp = stamp
        base_tf.header.frame_id = "odom"
        base_tf.child_frame_id = "base_link"
        base_tf.transform.translation.x = self._x
        base_tf.transform.translation.y = self._y
        base_tf.transform.rotation.z = qz
        base_tf.transform.rotation.w = qw
        self._tf.sendTransform([map_tf, base_tf])

        if not force_map:
            return
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = "map"
        grid.info.resolution = self._grid.resolution
        grid.info.width = self._grid.size
        grid.info.height = self._grid.size
        grid.info.origin.position.x = self._grid.origin
        grid.info.origin.position.y = self._grid.origin
        grid.info.origin.orientation.w = 1.0
        grid.data = self._grid.data.tolist()
        self._map_pub.publish(grid)


def main():
    rclpy.init()
    node = LidarLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
