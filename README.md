# SLAM & RL Resource Manager Workspace (ROS 2 Jazzy)

A workspace for SLAM, reactive exploration, and RL-based resource management on TurtleBot3.

## Components

- **`tb3_reactive_explorer`**: Obstacle avoidance and random wandering (`/scan` → `/cmd_vel`).
- **`mini_mapper`**: Simple occupancy-grid mapper (`/odom` + `/scan` → `/mini_map`).
- **`rl_resource_manager`**: Q-learning agent managing system resources (nice values). Supports persistent Q-table and epsilon decay.
- **`experiment_metrics.py`**: Benchmarking script for repeated learning episodes and overlapping load measurement.

## Setup & Build

```bash
# 1. Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# 2. Build the workspace
cd ~/slam_rl_ws
colcon build --symlink-install

# 3. Source the workspace
source ~/slam_rl_ws/install/setup.bash
```

## Running the System

Open multiple terminals to run the components.

### Terminal 1: Simulation (Gazebo)
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2: SLAM (SLAM Toolbox)
Basic launch:
```bash
source ~/slam_rl_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```
Or with custom simulation time and parameters:
```bash
source ~/slam_rl_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:=$HOME/slam_rl_ws/mapper_params_online_async.yaml
  
```
*(Note: `slam_params_file` dùng đường dẫn tuyệt đối nên có thể chạy ở bất kỳ thư mục nào.)*

### Terminal 3: Explorer Agent
Runs the reactive exploration node to fetch sensor data and drive the robot.
```bash
source ~/slam_rl_ws/install/setup.bash
ros2 run tb3_reactive_explorer reactive_explorer --ros-args \
  -p use_sim_time:=true \
  -p forward_speed:=0.15 \
  -p turn_speed:=0.5 \
  -p avoid_front:=0.5 \
  -p wander_turn_every_s:=10.0
```

### Terminal 4: RL Resource Manager (Root)
Run the resource manager with `sudo` to allow it to adjust process priorities (nice values).
The manager now saves/loads its Q-table from `~/slam_rl_ws/q_table.json` for persistent learning.

```bash
# Must use absolute path to setup.bash inside the root shell
sudo bash -c 'source /opt/ros/jazzy/setup.bash && source /home/tung/slam_rl_ws/install/setup.bash && ros2 run rl_resource_manager rl_resource_manager'
```

### Terminal 5: Visualization
```bash
ros2 run rviz2 rviz2
```
*Tip: In RViz, set Fixed Frame to `map` and add the `/map` and `/scan` topics.*

---

## Running Experiments (Training Mode)

Use the updated script to run repeated experiments for training the RL manager.
This supports **overlapping CPU hogs** and **multiple episodes**.

**Example:**
- **Loads**: [0.6, 0.8, 0.9]
- **Step**: Start a new hog every 10s.
- **Duration**: Each hog runs for 30s (creating overlap).
- **Episodes**: Repeat the sequence 5 times.
- **Wait**: Pause 20s between episodes.

```bash
source ~/slam_rl_ws/install/setup.bash
cd ~/slam_rl_ws

python3 experiment_metrics.py \
  --loads 0.6,0.8,0.9 \
  --step 10 \
  --hog-duration 30 \
  --episodes 5 \
  --episode-wait 20 \
  --outdir metrics_out
```

The script will produce a CSV and PNG in `metrics_out/` and print a summary table comparing performance across episodes.

### Saving the Map
To save the generated map into this repo (recommended):
```bash
source ~/slam_rl_ws/install/setup.bash
mkdir -p ~/slam_rl_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/slam_rl_ws/maps/my_map
```

## Nav2 (Load Map + Navigate)

After creating and saving a map with SLAM Toolbox, stop SLAM Toolbox and launch Nav2 with the saved map:

```bash
source ~/slam_rl_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/slam_rl_ws/maps/my_map.yaml
```

In RViz:
- Use **2D Pose Estimate** once to initialize AMCL.
- Use **Nav2 Goal** to send a navigation target.

### Dynamic Obstacles (Moving Objects)

This workspace includes a small helper package that spawns a few moving box obstacles in Gazebo Sim and moves them continuously. Nav2 should detect them via `/scan` and re-plan / avoid.

Run only the obstacle spawner/mover:
```bash
source ~/slam_rl_ws/install/setup.bash
ros2 run tb3_nav2_dynamic dynamic_obstacles --ros-args -p use_sim_time:=true
```

Or run a single launch that starts Gazebo + Nav2 + dynamic obstacles:
```bash
source ~/slam_rl_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch tb3_nav2_dynamic nav2_dynamic.launch.py map:=$HOME/slam_rl_ws/maps/my_map.yaml
```

If you already have Gazebo running, don’t start a second instance:
```bash
source ~/slam_rl_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch tb3_nav2_dynamic nav2_dynamic.launch.py \
  start_gazebo:=false \
  map:=$HOME/slam_rl_ws/maps/my_map.yaml
```

If you see errors like `Invalid frame ID "map"` or AMCL asking to set initial pose, either:
- Use RViz **2D Pose Estimate**, or
- Keep `auto_initial_pose:=true` (default) in `nav2_dynamic.launch.py` and adjust spawn pose if needed:
```bash
ros2 launch tb3_nav2_dynamic nav2_dynamic.launch.py \
  robot_x:=-2.0 robot_y:=-0.5 robot_yaw:=0.0 \
  map:=$HOME/slam_rl_ws/maps/my_map.yaml
```

If you see `Robot is out of bounds of the costmap!`, your `robot_x/robot_y` (and initial pose) is outside the saved map bounds. Set `robot_x/robot_y` to a point inside the map and restart.

Optional: send waypoints programmatically (Nav2 Simple Commander):
```bash
source ~/slam_rl_ws/install/setup.bash
ros2 run tb3_nav2_dynamic patrol_robot
```
