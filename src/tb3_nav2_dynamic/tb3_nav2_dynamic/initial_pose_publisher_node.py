import math

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def _yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("initial_pose_publisher")

        self.declare_parameter("frame_id", "map")
        # Match turtlebot3_world default spawn.
        self.declare_parameter("x", -2.0)
        self.declare_parameter("y", -0.5)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("delay_s", 2.0)
        self.declare_parameter("repeats", 20)
        self.declare_parameter("repeat_period_s", 0.5)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._x = float(self.get_parameter("x").value)
        self._y = float(self.get_parameter("y").value)
        self._yaw = float(self.get_parameter("yaw").value)
        self._delay_s = float(self.get_parameter("delay_s").value)
        self._remaining = int(self.get_parameter("repeats").value)
        self._repeat_period_s = float(self.get_parameter("repeat_period_s").value)

        self._pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._started = False
        self._timer = self.create_timer(max(self._repeat_period_s, 0.05), self._tick)
        self._start_time = self.get_clock().now()

    def _make_msg(self) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = Time()
        msg.header.frame_id = self._frame_id
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        qx, qy, qz, qw = _yaw_to_quat(self._yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        cov[0] = 0.25  # x
        cov[7] = 0.25  # y
        cov[35] = 0.068  # yaw (approx)
        msg.pose.covariance = cov
        return msg

    def _tick(self):
        elapsed_s = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        if not self._started:
            if elapsed_s < self._delay_s:
                return
            self._started = True
            self.get_logger().info(
                f"Publishing /initialpose: frame={self._frame_id} x={self._x} y={self._y} yaw={self._yaw}"
            )

        if self._remaining <= 0:
            self.get_logger().info("Initial pose published; exiting.")
            self._timer.cancel()
            return

        self._pub.publish(self._make_msg())
        self._remaining -= 1


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
