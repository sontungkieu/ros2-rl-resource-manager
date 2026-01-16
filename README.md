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
  slam_params_file:=/home/tung/slam_rl_ws/mapper_params_online_async.yaml
  
```
*(Note: Run this from the workspace root where the params file is located)*

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
To save the generated map to disk:
```bash
source ~/slam_rl_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```
