# My Car Bot

A 4-wheeled car robot simulation using ROS 2 Jazzy and Gazebo.

**Features:**
- Custom Fenced World
- Differential Drive (rear wheels)
- Lidar Sensor & SLAM Mapping
- Autonomous Navigation (Nav2)

---

## Prerequisites

```bash
# Install required packages
sudo apt update
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server ros-jazzy-robot-localization xterm
```

---

## Build

```bash
cd /media/a-qassem/01DBC69DA6BBEB20/FCIS/ROB/task-for-everyone/Ahmed\ Syam/my_robot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 1. Simulation Only (Manual Drive)

Launch the robot in Gazebo with teleop control:

```bash
cd /media/a-qassem/01DBC69DA6BBEB20/FCIS/ROB/task-for-everyone/Ahmed\ Syam/my_robot_ws
source install/setup.bash
ros2 launch my_car_bot sim.launch.py
```

**Controls (XTerm window):**
- `i` = Forward
- `,` = Backward  
- `j` = Turn Left
- `l` = Turn Right
- `k` = Stop

---

## 2. Mapping (SLAM)

Create a map of the environment:

```bash
cd /media/a-qassem/01DBC69DA6BBEB20/FCIS/ROB/task-for-everyone/Ahmed\ Syam/my_robot_ws
source install/setup.bash
ros2 launch my_car_bot mapping.launch.py
```

**In RViz:**
1. Set **Fixed Frame** to `map`
2. Add display: **Map** (topic: `/map`)
3. Add display: **LaserScan** (topic: `/scan`)
4. Drive around using XTerm to build the map

**Save the map (new terminal):**
```bash
cd /media/a-qassem/01DBC69DA6BBEB20/FCIS/ROB/task-for-everyone/Ahmed\ Syam/my_robot_ws
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f src/my_car_bot/maps/my_map --ros-args -p use_sim_time:=true
```

---

## 3. Autonomous Navigation (Nav2)

Navigate autonomously using the saved map:

### Step 1: Launch Navigation
```bash
cd /media/a-qassem/01DBC69DA6BBEB20/FCIS/ROB/task-for-everyone/Ahmed\ Syam/my_robot_ws
source install/setup.bash
ros2 launch my_car_bot navigation.launch.py
```

### Step 2: Activate Nav2 Lifecycle Nodes (if needed)
If navigation doesn't work immediately, run in a new terminal:
```bash
source /opt/ros/jazzy/setup.bash
ros2 lifecycle set /controller_server activate
ros2 lifecycle set /planner_server activate
ros2 lifecycle set /behavior_server activate
ros2 lifecycle set /bt_navigator configure
ros2 lifecycle set /bt_navigator activate
```

### Step 3: Navigate in RViz
1. Click **2D Pose Estimate** → Click & drag on map to set robot's initial position
2. Click **2D Goal Pose** → Click & drag on map to set destination
3. Robot will autonomously navigate to the goal!

---

## Quick Reference

| Task | Command |
|------|---------|
| Build | `colcon build --symlink-install` |
| Simulation | `ros2 launch my_car_bot sim.launch.py` |
| Mapping | `ros2 launch my_car_bot mapping.launch.py` |
| Save Map | `ros2 run nav2_map_server map_saver_cli -f src/my_car_bot/maps/my_map --ros-args -p use_sim_time:=true` |
| Navigation | `ros2 launch my_car_bot navigation.launch.py` |

---

## Troubleshooting

**Map not showing in RViz:**
- Set Fixed Frame to `map`
- Add Map display with topic `/map`

**Robot not moving during navigation:**
- Run the lifecycle activation commands (Step 2 above)
- Make sure to set initial pose with "2D Pose Estimate"

**LaserScan not showing:**
- Add LaserScan display with topic `/scan`
