import math
import os
import subprocess
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node


def _yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _box_sdf(size_xyz, rgba):
    sx, sy, sz = size_xyz
    r, g, b, a = rgba

    mass = 1.0
    ixx = (1.0 / 12.0) * mass * (sy * sy + sz * sz)
    iyy = (1.0 / 12.0) * mass * (sx * sx + sz * sz)
    izz = (1.0 / 12.0) * mass * (sx * sx + sy * sy)

    # Use a fixed model name in the SDF; the spawn service uses `EntityFactory.name`
    # to set the final name. Keeping this stable avoids template string substitution.
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="dynamic_obstacle_box">
    <static>false</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <pose>0 0 {0.5 * sz:.6f} 0 0 0</pose>
      <gravity>false</gravity>
      <inertial>
        <mass>{mass:.6f}</mass>
        <inertia>
          <ixx>{ixx:.6f}</ixx>
          <iyy>{iyy:.6f}</iyy>
          <izz>{izz:.6f}</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>{sx:.6f} {sy:.6f} {sz:.6f}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{sx:.6f} {sy:.6f} {sz:.6f}</size>
          </box>
        </geometry>
        <material>
          <ambient>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class DynamicObstacles(Node):
    def __init__(self):
        super().__init__("dynamic_obstacles")

        self.declare_parameter("world", "default")
        self.declare_parameter("update_rate_hz", 2.0)
        self.declare_parameter("spawn_obstacles", True)
        self.declare_parameter("size_x", 0.25)
        self.declare_parameter("size_y", 0.25)
        self.declare_parameter("size_z", 0.5)

        self._world = self.get_parameter("world").get_parameter_value().string_value.strip() or "default"

        self._size_xyz = (
            float(self.get_parameter("size_x").value),
            float(self.get_parameter("size_y").value),
            float(self.get_parameter("size_z").value),
        )

        self._spawn_obstacles = bool(self.get_parameter("spawn_obstacles").value)
        self._update_rate_hz = float(self.get_parameter("update_rate_hz").value)

        ros_gz_prefix = Path(get_package_prefix("ros_gz_sim"))
        self._create_exe = ros_gz_prefix / "lib" / "ros_gz_sim" / "create"
        self._set_entity_pose_exe = ros_gz_prefix / "lib" / "ros_gz_sim" / "set_entity_pose"

        cache_dir = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "tb3_nav2_dynamic"
        cache_dir.mkdir(parents=True, exist_ok=True)
        self._cache_dir = cache_dir
        self._pose_procs = {}

        self._obstacles = [
            {
                "name": "dyn_obstacle_1",
                "center": (-0.4, -0.2),
                "amp": (0.9, 0.0),
                "period": 8.0,
                "yaw": 0.0,
                "color": (0.9, 0.2, 0.2, 1.0),
            },
            {
                "name": "dyn_obstacle_2",
                "center": (0.4, 0.4),
                "amp": (0.0, 0.9),
                "period": 10.0,
                "yaw": 0.0,
                "color": (0.2, 0.2, 0.9, 1.0),
            },
            {
                "name": "dyn_obstacle_3",
                "center": (0.6, -0.6),
                "amp": (0.4, 0.4),
                "period": 12.0,
                "yaw": 0.0,
                "color": (0.2, 0.8, 0.2, 1.0),
                "mode": "circle",
            },
        ]

        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(1.0 / max(self._update_rate_hz, 1e-3), self._on_timer)

    def _run(self, argv, timeout_s: float) -> subprocess.CompletedProcess:
        return subprocess.run(argv, check=False, capture_output=True, text=True, timeout=timeout_s)

    def spawn(self) -> None:
        if not self._spawn_obstacles:
            self.get_logger().info("spawn_obstacles:=false; will only move existing entities.")
            return

        for obstacle in self._obstacles:
            x, y, z = self._obstacle_pos(t=0.0, obstacle=obstacle)

            sdf_path = self._cache_dir / f"{obstacle['name']}.sdf"
            sdf_path.write_text(_box_sdf(self._size_xyz, obstacle["color"]))

            argv = [
                str(self._create_exe),
                "-world",
                self._world,
                "-file",
                str(sdf_path),
                "-name",
                obstacle["name"],
                "-allow_renaming",
                "false",
                "-x",
                str(x),
                "-y",
                str(y),
                "-z",
                str(z),
            ]

            try:
                cp = self._run(argv, timeout_s=20.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"create timeout for '{obstacle['name']}'")
                continue

            if cp.returncode != 0:
                self.get_logger().warn(
                    f"create failed for '{obstacle['name']}' (maybe already exists). stderr={cp.stderr.strip()}"
                )

        self.get_logger().info(
            f"Dynamic obstacles ready. update_rate_hz={self._update_rate_hz}. "
            f"Using {self._create_exe.name} and {self._set_entity_pose_exe.name} (world='{self._world}')."
        )

    def _obstacle_pos(self, t: float, obstacle):
        cx, cy = obstacle["center"]
        ax, ay = obstacle["amp"]
        period = max(float(obstacle["period"]), 1e-3)
        mode = obstacle.get("mode", "line")

        phase = 2.0 * math.pi * (t / period)
        if mode == "circle":
            x = cx + ax * math.cos(phase)
            y = cy + ay * math.sin(phase)
        else:
            x = cx + ax * math.sin(phase)
            y = cy + ay * math.sin(phase)

        return (float(x), float(y), 0.0)

    def _move_obstacle(self, name: str, x: float, y: float, z: float, yaw: float) -> None:
        proc = self._pose_procs.get(name)
        if proc is not None and proc.poll() is None:
            # Previous set request still running; skip this tick to avoid process buildup.
            return

        argv = [
            str(self._set_entity_pose_exe),
            "--name",
            name,
            "--pos",
            str(x),
            str(y),
            str(z),
            "--euler",
            "0",
            "0",
            str(yaw),
        ]

        try:
            self._pose_procs[name] = subprocess.Popen(
                argv,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as exc:
            self.get_logger().warn(f"Failed to run set_entity_pose for '{name}': {exc}")

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        t = (now - self._start_time).nanoseconds * 1e-9

        for obstacle in self._obstacles:
            x, y, z = self._obstacle_pos(t=t, obstacle=obstacle)
            yaw = float(obstacle.get("yaw", 0.0))
            self._move_obstacle(obstacle["name"], x, y, z, yaw)


def main():
    rclpy.init()
    node = DynamicObstacles()
    try:
        node.spawn()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
