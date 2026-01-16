#!/usr/bin/env python3
import random
import json
import os
import atexit
from collections import deque, defaultdict
from typing import Optional, Tuple, Dict, List

import psutil
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


def find_pid_by_cmd_contains(substr: str) -> Optional[int]:
    s = substr.lower()
    for p in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
        try:
            name = (p.info.get("name") or "").lower()
            cmd = " ".join(p.info.get("cmdline") or []).lower()
            if s in name or s in cmd:
                return int(p.info["pid"])
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return None


def safe_set_nice(pid: int, nice_value: int, logger):
    try:
        psutil.Process(pid).nice(nice_value)
        logger.info(f"nice({pid})={nice_value}")
    except psutil.AccessDenied:
        logger.warn(f"no permission to set nice for PID={pid} (try sudo if you really need).")
    except psutil.NoSuchProcess:
        logger.warn(f"PID={pid} not found.")


class RLResourceManager(Node):
    """Minimal online Q-learning for resource management.

    Observations (state):
      - scan_ok (1/0) from /scan rate
      - map_ok (1/0) from /map period
      - cpu_hi (1/0) from total CPU percent

    Actions:
      0 BALANCED
      1 PRIORITIZE_SLAM (deprioritize cpu_hog + rviz2 + explorer)
      2 PRIORITIZE_VIZ  (keep rviz responsive; still limit cpu_hog)

    This gives you a clean OS story: RL changes process priority (nice) to protect SLAM.
    """

    def __init__(self):
        super().__init__("rl_resource_manager")
        # self.declare_parameter("use_sim_time", True)
        self.declare_parameter("step_s", 2.0)

        self.declare_parameter("min_scan_hz", 5.0)
        self.declare_parameter("max_map_period_s", 1.5)
        self.declare_parameter("cpu_high_pct", 75.0)

        self.declare_parameter("eps", 0.5)      # Start with high exploration
        self.declare_parameter("eps_min", 0.05)
        self.declare_parameter("eps_decay", 0.995)
        self.declare_parameter("alpha", 0.25)
        self.declare_parameter("gamma", 0.85)
        
        # Persistence
        self.declare_parameter("q_model_path", os.path.expanduser("~/slam_rl_ws/q_table.json"))

        self.scan_times = deque(maxlen=200)
        self.map_times = deque(maxlen=60)

        self.create_subscription(LaserScan, "/scan", self.on_scan, 50)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.create_subscription(String, "/cpu_hog_status", self.on_process_status, 10)

        self.Q: Dict[Tuple[int,int,int], List[float]] = defaultdict(lambda: [0.0, 0.0, 0.0])
        self.load_q_table()
        
        self.prev_state: Optional[Tuple[int,int,int]] = None
        self.prev_action: Optional[int] = None

        self.pid_hog = None     # From /cpu_hog_status
        self.pid_explorer = None  # From /cpu_hog_status (reactive_explorer)
        self.pid_rviz = None     # find_pid
        self.pid_gazebo = None   # find_pid

        self.create_timer(float(self.get_parameter("step_s").value), self.step)
        
        # Save on exit
        atexit.register(self.save_q_table)
        
        self.get_logger().info("rl_resource_manager started. Waiting /scan + /map + /cpu_hog_status ...")

    def load_q_table(self):
        path = self.get_parameter("q_model_path").value
        if os.path.exists(path):
            try:
                with open(path, "r") as f:
                    data = json.load(f)
                    count = 0
                    for k_str, v in data.items():
                        # key format "1,0,1"
                        try:
                            parts = k_str.split(',')
                            if len(parts) == 3:
                                key = (int(parts[0]), int(parts[1]), int(parts[2]))
                                self.Q[key] = v
                                count += 1
                        except:
                            pass
                self.get_logger().info(f"Loaded Q-table from {path} ({count} states)")
            except Exception as e:
                self.get_logger().error(f"Failed to load Q-table: {e}")
                
    def save_q_table(self):
        path = self.get_parameter("q_model_path").value
        try:
            # Convert keys to strings
            data = {}
            for k, v in self.Q.items():
                k_str = f"{k[0]},{k[1]},{k[2]}"
                data[k_str] = v
            
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            print(f"[rl_manager] Saved Q-table to {path}")
        except Exception as e:
            print(f"[rl_manager] Failed to save Q-table: {e}")

    def on_process_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            pid = data.get("pid")
            name = data.get("name", "")
            if not pid:
                return
            
            # Identify which component sent this
            if "cpu_hog" in name:
                self.pid_hog = pid
            elif "reactive_explorer" in name:
                self.pid_explorer = pid
        except json.JSONDecodeError:
            pass

    def on_scan(self, msg: LaserScan):
        self.scan_times.append(self.get_clock().now().nanoseconds / 1e9)

    def on_map(self, msg: OccupancyGrid):
        self.map_times.append(self.get_clock().now().nanoseconds / 1e9)

    def _rate_hz(self, times: deque) -> float:
        if len(times) < 2:
            return 0.0
        dt = times[-1] - times[0]
        return (len(times)-1) / dt if dt > 1e-6 else 0.0

    def _period_s(self, times: deque) -> float:
        if len(times) < 2:
            return float("inf")
        return times[-1] - times[-2]

    def _state(self) -> Tuple[int,int,int]:
        scan_hz = self._rate_hz(self.scan_times)
        map_period = self._period_s(self.map_times)
        cpu_total = psutil.cpu_percent(interval=None)

        min_scan = float(self.get_parameter("min_scan_hz").value)
        max_map_p = float(self.get_parameter("max_map_period_s").value)
        cpu_high = float(self.get_parameter("cpu_high_pct").value)

        scan_ok = 1 if scan_hz >= min_scan else 0
        map_ok = 1 if map_period <= max_map_p else 0
        cpu_hi = 1 if cpu_total >= cpu_high else 0

        self.get_logger().info(f"scan={scan_hz:.1f}Hz mapP={map_period:.2f}s cpu={cpu_total:.0f}% -> {scan_ok,map_ok,cpu_hi}")
        return (scan_ok, map_ok, cpu_hi)

    def _reward(self, state: Tuple[int,int,int]) -> float:
        scan_ok, map_ok, cpu_hi = state
        r = (1.0 if scan_ok else -1.0) + (1.0 if map_ok else -1.2) + (-0.3 if cpu_hi else 0.1)
        return r

    def _choose_action(self, state) -> int:
        eps = float(self.get_parameter("eps").value)
        if random.random() < eps:
            return random.randint(0, 2)
        q = self.Q[state]
        return int(max(range(3), key=lambda i: q[i]))
        
    def _decay_epsilon(self):
        eps = float(self.get_parameter("eps").value)
        eps_min = float(self.get_parameter("eps_min").value)
        eps_decay = float(self.get_parameter("eps_decay").value)
        
        if eps > eps_min:
            new_eps = max(eps_min, eps * eps_decay)
            # Set parameter back to ROS system
            self.set_parameters([rclpy.parameter.Parameter("eps", rclpy.Parameter.Type.DOUBLE, new_eps)])
            # Log periodically? (not every step)
            # self.get_logger().info(f"Epsilon decayed to {new_eps:.4f}")

    def _apply_action(self, action: int):
        # Discover PIDs that don't publish status (rviz, gazebo)
        if self.pid_rviz is None:
            self.pid_rviz = find_pid_by_cmd_contains("rviz2")
        # Try finding gazebo related processes
        if self.pid_gazebo is None:
             # Common gazebo process names
            p = find_pid_by_cmd_contains("gzserver") or \
                find_pid_by_cmd_contains("gzsim") or \
                find_pid_by_cmd_contains("ruby") # often launches gazebo
            if p: self.pid_gazebo = p

        # Action 0: BALANCED
        # Reset everything to default nice=0
        if action == 0:
            if self.pid_hog: safe_set_nice(self.pid_hog, 0, self.get_logger())
            if self.pid_rviz: safe_set_nice(self.pid_rviz, 0, self.get_logger())
            if self.pid_explorer: safe_set_nice(self.pid_explorer, 0, self.get_logger())
            if self.pid_gazebo: safe_set_nice(self.pid_gazebo, 0, self.get_logger())
            self.get_logger().info("action 0 BALANCED")

        # Action 1: PRIORITIZE_SLAM
        # Making others nicer (higher value = lower priority) to favor SLAM (which stays at 0 or effectively higher)
        elif action == 1:
            if self.pid_hog: safe_set_nice(self.pid_hog, 19, self.get_logger())  # Max nice
            if self.pid_rviz: safe_set_nice(self.pid_rviz, 15, self.get_logger())
            if self.pid_explorer: safe_set_nice(self.pid_explorer, 10, self.get_logger())
            if self.pid_gazebo: safe_set_nice(self.pid_gazebo, 5, self.get_logger()) # Slow down sim slightly if needed
            self.get_logger().info("action 1 PRIORITIZE_SLAM")

        # Action 2: PRIORITIZE_VIZ 
        # Keep rviz responsive, limit hogs heavily, explorer moderately
        else:
            if self.pid_hog: safe_set_nice(self.pid_hog, 15, self.get_logger())
            if self.pid_rviz: safe_set_nice(self.pid_rviz, -5, self.get_logger()) # Boost priority (needs sudo/privileges usually, else 0)
            if self.pid_explorer: safe_set_nice(self.pid_explorer, 5, self.get_logger())
            if self.pid_gazebo: safe_set_nice(self.pid_gazebo, 5, self.get_logger())
            self.get_logger().info("action 2 PRIORITIZE_VIZ")

    def step(self):
        state = self._state()
        reward = self._reward(state)

        if self.prev_state is not None and self.prev_action is not None:
            alpha = float(self.get_parameter("alpha").value)
            gamma = float(self.get_parameter("gamma").value)
            qsa = self.Q[self.prev_state][self.prev_action]
            self.Q[self.prev_state][self.prev_action] = qsa + alpha * (reward + gamma * max(self.Q[state]) - qsa)

        self._decay_epsilon()
        action = self._choose_action(state)
        self._apply_action(action)

        self.prev_state = state
        self.prev_action = action

def main():
    rclpy.init()
    node = RLResourceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
