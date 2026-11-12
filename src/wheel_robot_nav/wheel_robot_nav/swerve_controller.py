"""Convert /cmd_vel into steering positions and drive velocities for Isaac Sim.

A new translation or yaw is two steps. All four steer joints are sent to the
heading for that twist first. Rolling speed stays at zero until each measured
steer angle is on that heading, then the drive velocities turn on together.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

from wheel_robot_nav.kinematics import MODULES, steer_error, twist_to_wheels


class SwerveController(Node):
    def __init__(self):
        super().__init__("swerve_controller")
        self.declare_parameter("max_drive", 15.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("steer_tolerance", 0.08)
        qos = QoSProfile(depth=10)
        self._steer = [0.0, 0.0, 0.0, 0.0]
        self._measured = [None, None, None, None]
        self._twist = Twist()
        self._twist_time = self.get_clock().now()
        self._steer_pub = self.create_publisher(JointState, "/isaac_wheel_steer", qos)
        self._drive_pub = self.create_publisher(JointState, "/isaac_wheel_drive", qos)
        self.create_subscription(Twist, "/cmd_vel", self._on_twist, qos)
        self.create_subscription(JointState, "/isaac_joint_states", self._on_joints, qos)
        self.create_timer(0.02, self._tick)

    def _on_twist(self, msg):
        self._twist = msg
        self._twist_time = self.get_clock().now()

    def _on_joints(self, msg):
        index = {}
        for name_index, name in enumerate(msg.name):
            index[name] = name_index
            index[name.rsplit("/", 1)[-1]] = name_index
        for module_index, (steer_name, _, _, _, _) in enumerate(MODULES):
            joint_index = index.get(steer_name)
            if joint_index is None or joint_index >= len(msg.position):
                continue
            self._measured[module_index] = msg.position[joint_index]

    def _wheels_aligned(self, targets):
        tolerance = float(self.get_parameter("steer_tolerance").value)
        for measured, target in zip(self._measured, targets):
            if measured is None or steer_error(measured, target) > tolerance:
                return False
        return True

    def _tick(self):
        now = self.get_clock().now()
        age = (now - self._twist_time).nanoseconds * 1.0e-9
        timeout = float(self.get_parameter("command_timeout").value)
        twist = self._twist if age < timeout else Twist()
        max_drive = float(self.get_parameter("max_drive").value)
        # Previous command is the reference, so each wheel keeps the heading
        # already chosen for this twist instead of flipping while it is moving.
        targets, drives = twist_to_wheels(
            twist.linear.x, twist.linear.y, twist.angular.z, self._steer
        )
        self._steer = targets
        if not self._wheels_aligned(targets):
            drives = [0.0, 0.0, 0.0, 0.0]
        else:
            drives = [max(-max_drive, min(max_drive, value)) for value in drives]
        stamp = now.to_msg()

        steer = JointState()
        steer.header.stamp = stamp
        steer.name = [module[0] for module in MODULES]
        steer.position = list(self._steer)
        self._steer_pub.publish(steer)

        drive = JointState()
        drive.header.stamp = stamp
        drive.name = [module[1] for module in MODULES]
        drive.velocity = drives
        self._drive_pub.publish(drive)


def main():
    rclpy.init()
    node = SwerveController()
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
