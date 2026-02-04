import argparse
import math
from typing import Iterable, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


def _yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _pose_stamped(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    qx, qy, qz, qw = _yaw_to_quat(float(yaw))
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def _parse_waypoints(values: Iterable[str]) -> Tuple[Tuple[float, float, float], ...]:
    floats = [float(v) for v in values]
    if len(floats) % 3 != 0:
        raise ValueError("--waypoints must be a flat list of x y yaw triples")
    triples = []
    for i in range(0, len(floats), 3):
        triples.append((floats[i], floats[i + 1], floats[i + 2]))
    return tuple(triples)


def main():
    ap = argparse.ArgumentParser(description="Send Nav2 waypoints (Nav2 Simple Commander).")
    ap.add_argument("--frame", default="map", help="Frame for waypoints (default: map)")
    ap.add_argument(
        "--waypoints",
        nargs="*",
        default=None,
        help="Flat list of x y yaw (radians) triples, e.g. --waypoints 0 0 0 1 0 0",
    )
    args = ap.parse_args()

    if args.waypoints is None:
        waypoints = (
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 1.57),
            (0.0, 1.0, 3.14),
        )
    else:
        waypoints = _parse_waypoints(args.waypoints)
        if not waypoints:
            raise SystemExit("No waypoints provided.")

    rclpy.init()
    nav = BasicNavigator()
    try:
        nav.waitUntilNav2Active()

        stamped = [_pose_stamped(args.frame, x, y, yaw) for (x, y, yaw) in waypoints]
        nav.followWaypoints(stamped)

        while rclpy.ok() and not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.1)

        result = nav.getResult()
        print(f"Nav2 result: {result}")
    finally:
        rclpy.shutdown()

