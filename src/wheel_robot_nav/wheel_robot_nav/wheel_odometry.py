"""Integrate wheel joint states into /odom and the odom -> base_link transform."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from wheel_robot_nav.kinematics import MODULES, wheels_to_twist


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        qos = QoSProfile(depth=20)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_stamp = None
        self._steer = {name: 0.0 for name, _, _, _, _ in MODULES}
        self._drive = {name: None for _, name, _, _, _ in MODULES}
        self._pub = self.create_publisher(Odometry, "/odom", qos)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(JointState, "/isaac_joint_states", self._on_joints, qos)

    def _on_joints(self, msg):
        names = set(msg.name)
        for index, name in enumerate(msg.name):
            if name in self._steer and index < len(msg.position):
                self._steer[name] = msg.position[index]
            if name in self._drive and index < len(msg.velocity):
                self._drive[name] = msg.velocity[index]
        if any(self._drive[name] is None for _, name, _, _, _ in MODULES):
            return
        if not all(name in names for name, _, _, _, _ in MODULES):
            return

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self._last_stamp is None:
            self._last_stamp = stamp
            return
        dt = (stamp - self._last_stamp).nanoseconds * 1.0e-9
        self._last_stamp = stamp
        if dt <= 0.0 or dt > 0.5:
            return

        steer = [self._steer[name] for name, _, _, _, _ in MODULES]
        drive = [self._drive[name] for _, name, _, _, _ in MODULES]
        vx, vy, wz = wheels_to_twist(steer, drive)
        cy = math.cos(self._yaw)
        sy = math.sin(self._yaw)
        self._x += (vx * cy - vy * sy) * dt
        self._y += (vx * sy + vy * cy) * dt
        self._yaw += wz * dt

        odom_frame = self.get_parameter("odom_frame").value
        base_frame = self.get_parameter("base_frame").value
        half = 0.5 * self._yaw
        qz = math.sin(half)
        qw = math.cos(half)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.05
        self._pub.publish(odom)

        if self.get_parameter("publish_tf").value:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = odom_frame
            tf.child_frame_id = base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf.sendTransform(tf)


def main():
    rclpy.init()
    node = WheelOdometry()
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
