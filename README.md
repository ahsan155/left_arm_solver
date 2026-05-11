# MoveIt Arm Solver

A ROS 2 motion planning solver for the **Unitree G1 humanoid robot left arm**. Given a sequence of Cartesian waypoints, the node plans collision-free joint trajectories using MoveIt 2, streams the intermediate trajectory points to the robot, and waits at each waypoint before moving to the next.

---

## Description

### Planning Pipeline

The solver uses **MoveIt 2** — the standard ROS 2 motion planning framework — to drive the G1's `left_arm` planning group (7 DOF, `torso_link` → `left_wrist_yaw_link`).

**OMPL (Open Motion Planning Library)** is the underlying planning algorithm. OMPL is a sampling-based planner; by default the solver is configured to use the OMPL pipeline, which explores the joint-space configuration randomly and finds a collision-free path to the goal. Key planning parameters (number of attempts, time limit, velocity/acceleration scaling) are tunable via a YAML config file.

**Time parameterisation** is applied by MoveIt after the geometric path is found. This converts the raw path into a timed trajectory with velocity and acceleration profiles that respect the joint limits.

### Solver State Machine

The node runs an internal state machine on a 50 ms timer:

1. **WAIT_JOINT_STATES** — waits until all 7 left-arm joint positions arrive on the input topic.
2. **CAPTURE_EE_ORIENTATION** — calls the MoveIt `/compute_fk` service to record the end-effector orientation at the start pose. This orientation is held fixed throughout all waypoints (position-only Cartesian goals).
3. **PLAN_WAYPOINT** — calls `/plan_kinematic_path` with a position + orientation constraint for the current waypoint. On success the planned trajectory is streamed to the output topic (and optionally sent to a `ros2_control` trajectory controller in simulation).
4. **HOLD** — the arm holds position at the waypoint for a configurable duration before advancing.
5. **FINISHED** — all waypoints have been reached.

### Trajectory Streaming

After planning succeeds, the node does not just send the final joint position. It replays every intermediate trajectory point at the configured command rate (~20 Hz), producing smooth motion on the real robot instead of a single step-change.

### RViz Visualisation

The solver publishes:
- The planned trajectory to `/display_planned_path` for the MoveIt "Planned Path" panel.
- A green highlighted robot state to `/display_start_robot_state`.
- An orange highlighted robot state to `/display_goal_robot_state`.

---

## Inputs and Outputs

### ROS Topics

| Direction | Topic | Message Type | Description |
|-----------|-------|-------------|-------------|
| Subscribe | `/joint_states_ordered` | `sensor_msgs/JointState` | Current joint positions in canonical order. Published by the `ordered_joint_states_bridge` node from the raw `/joint_states` source. |
| Publish | `/arm_joint_cmd` | `std_msgs/Float32MultiArray` | 17-element array of joint position commands in `UNITREE_CMD_JOINTS` order (see table below). Streamed at ~20 Hz. |
| Publish | `/display_planned_path` | `moveit_msgs/DisplayTrajectory` | Full planned trajectory for RViz visualisation. |
| Publish | `/display_start_robot_state` | `moveit_msgs/DisplayRobotState` | Current start state highlighted green in RViz. |
| Publish | `/display_goal_robot_state` | `moveit_msgs/DisplayRobotState` | Current goal state highlighted orange in RViz. |

### MoveIt Services (provided by `move_group`)

| Direction | Service / Action | Type | Description |
|-----------|-----------------|------|-------------|
| Client | `/plan_kinematic_path` | `moveit_msgs/GetMotionPlan` | Requests a collision-free joint trajectory to the goal pose. |
| Client | `/compute_fk` | `moveit_msgs/GetPositionFK` | Reads end-effector orientation at the start configuration. |
| Client (sim only) | `/left_arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Sends the planned trajectory to the `ros2_control` controller so the Scene Robot moves in RViz. Disabled on the real robot (`execute_via_controller: false`). |

### Defining Waypoints

Target Cartesian waypoints are defined in
[`left_arm_solver/joint_table_target.py`](left_arm_solver/joint_table_target.py)
as the `target_waypoints` list. Each entry is an `(x, y, z)` tuple expressing the desired
end-effector (`left_wrist_yaw_link`) position in the `torso_link` frame (metres).

```python
# Example entries from joint_table_target.py
target_waypoints = [
    (0.3, base_y, 0.05),   # forward, low
    (0.3, base_y, 0.25),   # forward, mid
    (0.3, base_y, 0.45),   # forward, high
    ...
]
```

Known reachable workspace (relative to `torso_link`):

| Axis | Min (m) | Max (m) | Note |
|------|---------|---------|------|
| x | 0.19 | 0.34 | forward reach |
| y | 0.14 | 0.41 | right → left |
| z | 0.001 | 0.50 | lowest → highest |

### `/arm_joint_cmd` Joint Order and Limits

The 17-element output array maps to the following joints:

| Index | Joint Name | Min (rad) | Max (rad) |
|-------|-----------|-----------|-----------|
| 0 | `waist_yaw_joint` | -2.500 | 2.500 |
| 1 | `waist_roll_joint` | -0.520 | 0.520 |
| 2 | `waist_pitch_joint` | -0.520 | 0.520 |
| 3 | `left_shoulder_pitch_joint` | -2.900 | 2.500 |
| 4 | `left_shoulder_roll_joint` | -1.400 | 2.100 |
| 5 | `left_shoulder_yaw_joint` | -2.500 | 2.500 |
| 6 | `left_elbow_joint` | -0.900 | 2.000 |
| 7 | `left_wrist_roll_joint` | -1.800 | 1.800 |
| 8 | `left_wrist_pitch_joint` | -1.614 | 1.614 |
| 9 | `left_wrist_yaw_joint` | -1.614 | 1.614 |
| 10 | `right_shoulder_pitch_joint` | -2.900 | 2.500 |
| 11 | `right_shoulder_roll_joint` | -2.100 | 1.400 |
| 12 | `right_shoulder_yaw_joint` | -2.500 | 2.500 |
| 13 | `right_elbow_joint` | -0.900 | 2.000 |
| 14 | `right_wrist_roll_joint` | -1.800 | 1.800 |
| 15 | `right_wrist_pitch_joint` | -1.614 | 1.614 |
| 16 | `right_wrist_yaw_joint` | -1.614 | 1.614 |

Only the 7 left-arm joints (indices 3–9) are assigned planned values; all other slots are set to `0.0` (hold position). All values are clamped to the limits above before publishing.

---

## Prerequisites

**Operating System:** Ubuntu 22.04 LTS

**ROS 2 Humble Hawksbill** must be installed before anything else.
Follow the official installation guide: https://docs.ros.org/en/humble/Installation.html

After installing ROS 2, install the required build tools:

```bash
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool

sudo rosdep init
rosdep update

colcon mixin add default \
  https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

Remove any pre-installed MoveIt 2 Debian packages — they conflict with a source build:

```bash
sudo apt remove ros-humble-moveit*
```

---

## Build Project

> **Note:** MoveIt 2 is built from source. The build takes 20–30 minutes depending on your machine. 32 GB of RAM is recommended; on lower-memory systems append `--parallel-workers 2` to the `colcon build` command.

### 1. Create the workspace

```bash
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2/src
```

### 2. Clone MoveIt 2 source and pull all dependencies

Clone MoveIt 2 and use its `.repos` file to pull the full dependency set — this includes `moveit_msgs`, `moveit_resources`, and all other packages that do not come with the tutorials repo ([source build guide](https://moveit.ai/install-moveit2/source/)):

```bash
export ROS_DISTRO=humble
git clone https://github.com/moveit/moveit2.git -b $ROS_DISTRO
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
```

### 3. Clone MoveIt 2 tutorials and pull tutorial dependencies

The tutorials repository adds additional demo packages and a supplementary `.repos` file:

```bash
git clone -b humble https://github.com/moveit/moveit2_tutorials
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
```

### 4. Clone this repository

```bash
git clone https://github.com/<your-org>/left_arm_solver
```

Replace `<your-org>` with the actual GitHub organisation or user name.

### 5. Install ROS dependencies

```bash
cd ~/ws_moveit2
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

### 6. Build

```bash
source /opt/ros/humble/setup.bash
colcon build --mixin release
```

### 7. Source the workspace

```bash
source ~/ws_moveit2/install/setup.bash
```

Add this line to `~/.bashrc` to avoid sourcing it manually on every new terminal:

```bash
echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc
```

---

## Launch Files

| Launch File | Purpose |
|------------|---------|
| `moveit_demo.launch.py` | Full simulation stack: `robot_state_publisher`, `move_group`, `ros2_control` mock hardware, controllers, RViz. |
| `moveit_real.launch.py` | Real robot stack: `robot_state_publisher`, `move_group`, RViz only. No mock hardware or controllers. |
| `waypoint_node.launch.py` | Starts the solver node. Use `execute_via_controller:=false` with the real robot. |

### Simulation

```bash
# Terminal 1 — MoveIt + simulation hardware
ros2 launch left_arm_solver moveit_demo.launch.py

# Terminal 2 — Waypoint solver
ros2 launch left_arm_solver waypoint_node.launch.py
```

### Real Robot

```bash
# Terminal 1 — MoveIt stack (no mock hardware)
ros2 launch left_arm_solver moveit_real.launch.py

# Terminal 2 — Waypoint solver (streams to /arm_joint_cmd)
ros2 launch left_arm_solver waypoint_node.launch.py execute_via_controller:=false
```

The real robot hardware driver must be running and publishing `/joint_states` for all 29 body joints before launching.

---

## Configuration

All solver parameters are in [`config/waypoint_node.yaml`](config/waypoint_node.yaml).
Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `planning_time_sec` | `10.0` | Max seconds OMPL may spend per waypoint |
| `num_planning_attempts` | `8` | Independent planning attempts per waypoint |
| `max_velocity_scaling` | `0.4` | Fraction of joint velocity limits to use |
| `max_acceleration_scaling` | `0.4` | Fraction of joint acceleration limits to use |
| `hold_at_waypoint_sec` | `3.0` | Seconds to hold at each waypoint before advancing |
| `position_goal_radius_m` | `0.02` | Acceptable position error sphere radius (m) |
| `orientation_tolerance_rad` | `0.05` | Acceptable orientation error per axis (rad) |
| `cmd_publish_hz` | `20.0` | `/arm_joint_cmd` publish rate and trajectory streaming rate |
| `execute_via_controller` | `true` | Send trajectory to `ros2_control` controller (simulation only) |
