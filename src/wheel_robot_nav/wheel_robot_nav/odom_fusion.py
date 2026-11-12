"""Fuse /odom and /imu into /odometry/filtered and odom -> base_link."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from wheel_robot_nav.planar_ekf import PlanarEkf, quat_valid, relative_yaw, wrap


class OdomFusion(Node):
    def __init__(self):
        super().__init__("odom_fusion")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("use_imu_orientation", True)
        qos = QoSProfile(depth=20)
        self._filter = PlanarEkf()
        self._last = None
        self._q0 = None
        self._yaw0 = 0.0
        self._pub = self.create_publisher(Odometry, "/odometry/filtered", qos)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.create_subscription(Imu, "/imu", self._on_imu, qos)

    def _seconds(self, stamp):
        return rclpy.time.Time.from_msg(stamp).nanoseconds * 1.0e-9

    def _advance(self, stamp):
        if self._last is None:
            self._last = stamp
            return False
        dt = stamp - self._last
        if dt <= 0.0 or dt > 0.5:
            if dt > 0.5:
                self._last = stamp
            return False
        self._filter.predict(dt)
        self._last = stamp
        return True

    def _on_odom(self, msg):
        stamp = self._seconds(msg.header.stamp)
        twist = msg.twist.twist
        if self._last is None:
            q = msg.pose.pose.orientation
            self._filter.x[0] = msg.pose.pose.position.x
            self._filter.x[1] = msg.pose.pose.position.y
            self._filter.x[2] = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            self._filter.x[3] = twist.linear.x
            self._filter.x[4] = twist.linear.y
            self._filter.x[5] = twist.angular.z
            self._last = stamp
            self._publish(msg.header.stamp)
            return
        if not self._advance(stamp):
            return
        self._filter.update_twist(twist.linear.x, twist.linear.y, twist.angular.z)
        self._publish(msg.header.stamp)

    def _on_imu(self, msg):
        stamp = self._seconds(msg.header.stamp)
        if self._last is None:
            self._last = stamp
        elif not self._advance(stamp):
            return
        self._filter.update_gyro(msg.angular_velocity.z)
        q = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        if self.get_parameter("use_imu_orientation").value and quat_valid(q):
            if self._q0 is None:
                self._q0 = q
                self._yaw0 = self._filter.x[2]
            else:
                measured = wrap(self._yaw0 + relative_yaw(q, self._q0))
                self._filter.update_yaw(measured)
        self._publish(msg.header.stamp)

    def _publish(self, stamp):
        odom_frame = self.get_parameter("odom_frame").value
        base_frame = self.get_parameter("base_frame").value
        x, y, yaw, vx, vy, wz = self._filter.x
        half = 0.5 * yaw
        qz = math.sin(half)
        qw = math.cos(half)
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        pose_cov[0] = self._filter.P[0][0]
        pose_cov[7] = self._filter.P[1][1]
        pose_cov[35] = self._filter.P[2][2]
        twist_cov[0] = self._filter.P[3][3]
        twist_cov[7] = self._filter.P[4][4]
        twist_cov[35] = self._filter.P[5][5]

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = pose_cov
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = twist_cov
        self._pub.publish(odom)

        if not self.get_parameter("publish_tf").value:
            return
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = odom_frame
        tf.child_frame_id = base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
