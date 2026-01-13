# slam_rl_ws (ROS2 Jazzy)
Minimal workspace for SLAM + reactive exploration + RL-based resource management. Components:
- `tb3_reactive_explorer`: `/scan` → `/cmd_vel` obstacle avoidance + wandering.
- `mini_mapper`: demo occupancy-grid mapper using `/odom` + `/scan`, publishes `/mini_map`.
- `rl_resource_manager`: online Q-learning that changes OS nice of `cpu_hog`, `rviz2`, `reactive_explorer` to protect SLAM; includes `cpu_hog` node to create contention.
- `experiment_metrics.py`: standalone script to measure `/scan` and `/map` rates, jitter, stamp age, and CPU% while spinning up multiple `cpu_hog` loads, writing CSV + plot.

## Build
```bash
mkdir -p ~/slam_rl_ws/src
# copy packages into ~/slam_rl_ws/src
cd ~/slam_rl_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/slam_rl_ws/install/setup.bash
```

## Run (typical TurtleBot3 Gazebo)
Terminal 1 (sim):
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Terminal 2 (SLAM):
```bash
ros2 launch slam_toolbox online_async_launch.py
```

Terminal 3 (explorer):
```bash
ros2 run tb3_reactive_explorer reactive_explorer
# or with params:
ros2 run tb3_reactive_explorer reactive_explorer --ros-args \
  -p use_sim_time:=true \
  -p forward_speed:=0.10 \
  -p turn_speed:=0.45 \
  -p avoid_front:=0.65 \
  -p avoid_side:=0.50 \
  -p wander_turn_every_s:=12.0 \
  -p wander_turn_duration_s:=0.8
```

Terminal 4 (optional mini mapper):
```bash
ros2 run mini_mapper mini_mapper
```

Terminal 5 (optional RL resource manager + hog):
```bash
ros2 run rl_resource_manager cpu_hog --ros-args -p load:=0.9
ros2 run rl_resource_manager rl_resource_manager
```

RViz: `ros2 run rviz2 rviz2`

## Experiment metrics
Measure topic performance while gradually adding CPU load.
```bash
cd ~/slam_rl_ws
python3 experiment_metrics.py \
  --loads 0.6,0.8,0.9 \
  --start-after 10 \
  --step 20 \
  --duration 120 \
  --log-period 0.2 \
  --window 5.0 \
  --outdir metrics_out \
  --slam-regex "slam_toolbox" \
  --bridge-regex "ros_gz_bridge"
```
- Spawns `cpu_hog` nodes over time, tracks `/scan` and `/map` rate/jitter, stamp age, CPU% (slam_toolbox, ros_gz_bridge, hog total).
- Outputs CSV and PNG plot to `metrics_out/<timestamp>.{csv,png}` (use `--prefix` to override). Add `--dry-run` to skip starting hogs but still log topic stats.

## Save map (from `/map`)
```bash
mkdir -p ~/robot_maps
ros2 run nav2_map_server map_saver_cli -f ~/robot_maps/my_map
```

## RViz tips
- For raw scan test: Fixed Frame `odom`, add `/scan` LaserScan.
- For SLAM map: Fixed Frame `map`, add `/map` Map display.
