"""Fill IMU covariance so robot_localization will accept the message.

Isaac Sim's IMU publisher leaves the covariance arrays at zero, and
``ekf_node`` drops any measurement whose variance is zero.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu


def _diag(xx, yy, zz):
    return [xx, 0.0, 0.0, 0.0, yy, 0.0, 0.0, 0.0, zz]


class ImuCovariance(Node):
    def __init__(self):
        super().__init__("imu_covariance")
        qos = QoSProfile(depth=20)
        self._pub = self.create_publisher(Imu, "/imu/data", qos)
        self.create_subscription(Imu, "/imu", self._on_imu, qos)

    def _on_imu(self, msg):
        if msg.orientation_covariance[0] == 0.0:
            msg.orientation_covariance = _diag(0.05, 0.05, 0.02)
        if msg.angular_velocity_covariance[0] == 0.0:
            msg.angular_velocity_covariance = _diag(0.01, 0.01, 0.01)
        if msg.linear_acceleration_covariance[0] == 0.0:
            msg.linear_acceleration_covariance = _diag(0.2, 0.2, 0.2)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ImuCovariance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
