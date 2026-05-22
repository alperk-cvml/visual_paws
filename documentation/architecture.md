# Visual Paws — System Architecture

## Overview

Visual Paws is a modular ROS 2 quadruped robot simulation and control stack. It models a 12-DOF quadruped (3 joints per leg × 4 legs) running in Gazebo Harmonic, with perception, gait control, elevation mapping, and keyboard teleoperation. The entire system runs in Docker containers that communicate over the host network via ROS 2 DDS.

---

## Container Architecture

The root `docker-compose.yaml` uses Compose `include:` to aggregate six independently-defined service modules. Each module lives in its own subdirectory under `src/` with its own `Dockerfile` and `docker-compose.yaml`.

```
docker-compose.yaml (root)
 ├── include: src/quadruped_gazebo/docker-compose.yaml
 ├── include: src/quadruped_gait_controller/docker-compose.yaml
 ├── include: src/quadruped_test_controller/docker-compose.yaml
 ├── include: src/dept_image_proc/docker-compose.yaml
 ├── include: src/elevation_mapping_cupy_pkg/docker-compose.yaml
 └── include: src/nav2_mppi_navigation/docker-compose.yaml
```

All containers share:
- **Network**: `host` — zero-config ROS 2 DDS (CycloneDDS) discovery
- **IPC**: `host` — low-latency inter-process shared memory
- **Workspace**: repo root mounted at `/work/ws` (ROS 2 workspace)
- **ROS distribution**: Jazzy (Ubuntu 24.04)

### Container Summary

| Container | Base Image | Role |
|---|---|---|
| `quadruped_gazebo` | `osrf/ros:jazzy-desktop-full` | Physics simulation, robot model, sensor data |
| `quadruped_gait_controller` | `osrf/ros:jazzy-desktop-full` | Inverse kinematics + diagonal trot gait |
| `quadruped_test_controller` | `osrf/ros:jazzy-desktop-full` | Keyboard teleoperation |
| `depth_image_proc` | `osrf/ros:jazzy-desktop-full` | Depth image → point cloud conversion |
| `elevation_mapping_cupy` | `nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04` | GPU-accelerated 2.5D elevation mapping |
| `nav2_mppi_navigation` | `osrf/ros:jazzy-desktop-full` | Nav2 stack with MPPI controller for autonomous navigation |

---

## ROS 2 Package Structure

```
src/
├── quadruped_gazebo/              # C++ CMake package — simulation
├── quadruped_gait_controller/     # Python package — gait control
├── quadruped_test_controller/     # Python package — keyboard teleop
├── dept_image_proc/               # Thin wrapper — depth processing
├── elevation_mapping_cupy_pkg/    # Python package — GPU elevation map
│   └── repo/                      # Vendored elevation_mapping_cupy repo
├── nav2_mppi_navigation/          # Python package — Nav2 + MPPI autonomous navigation
│   └── nav2_mppi_navigation/
│       ├── config/
│       │   ├── nav2_mppi_params.yaml  # Full Nav2 + MPPI parameter file
│       │   └── nav2.rviz              # RViz2 layout with Nav2 panel
│       └── launch/
│           └── nav2_mppi.launch.py    # Nav2 bringup + fake localisation
└── elevation_mapping/             # Legacy (inactive) elevation mapping
```

---

## Robot Model

The robot is defined in `src/quadruped_gazebo/urdf/quadruped.urdf.xacro`.

- **Body**: 0.25 m × 0.12 m, 1.2 kg
- **Legs**: 4 legs × 3 joints = 12 DOF total

### Joint Layout (per leg)

```
body
 └── hip_roll    (frontal plane abduction/adduction)
      └── hip_pitch  (sagittal plane flexion)
           └── knee      (sagittal plane flexion)
                └── foot
```

**Leg link lengths**: thigh 85 mm · calf 95 mm  
**Leg segment masses**: ~0.08–0.10 kg each

### Sensors

| Sensor | Topic prefix | Purpose |
|---|---|---|
| Front RGB camera | `/front_camera/` | Visual reference |
| Depth camera | `/depth_camera/` | 3D perception for elevation mapping |
| IMU | `/imu/data` | Orientation and acceleration |

---

## Data Flow

```
┌──────────────────────────────────────┐
│         Gazebo Simulation            │
│  - quadruped physics                 │
│  - joint state publishing            │
│  - sensor simulation                 │
└────────────┬────────────┬────────────┘
             │            │
      /depth_camera/*   /tf, /imu/data
      /front_camera/*  /joint_states
             │
             ▼
┌────────────────────┐
│  Depth Image Proc  │
│  depth → points    │
└────────┬───────────┘
         │ /depth_camera/points
         ▼
┌────────────────────────┐
│  Elevation Mapping     │
│  CuPy (GPU)            │
│  3m × 3m, 5cm res      │
│  2 Hz update           │
└────────┬───────────────┘
         │ /elevation_map
         │ /occupancy_grid  ◀── consumed by Nav2 costmaps
         ▼
┌──────────────────────────────────────────┐
│          Nav2 MPPI Navigation            │
│  - BT Navigator (navigate_to_pose)       │
│  - NavFn global planner                  │
│  - MPPI controller (20 Hz)               │
│  - Local + global costmap (from          │
│    /occupancy_grid, 4m×4m, 5cm res)      │
│  - Fake localisation (map == odom)       │
└────────────────┬─────────────────────────┘
                 │ /cmd_vel  (collision_monitor → velocity_smoother)
                 ▼
┌────────────────────────┐        ┌────────────────────────┐
│  Keyboard Controller   │        │    Gait Controller     │
│  W/A/S/D/Q/E → Twist  │───────▶│  /cmd_vel → IK → cmds  │
└────────────────────────┘        └──────────┬─────────────┘
  (manual override; same /cmd_vel topic)      │
                              /quadruped_joint_controller/commands
                                             │
                                             ▼
                                   Gazebo joint actuators
```

---

## Gait Controller

**File**: `src/quadruped_gait_controller/quadruped_gait_controller/gait_node.py`

The controller implements a **diagonal trot gait** with analytic 3-DOF inverse kinematics.

### Gait Parameters

| Parameter | Value |
|---|---|
| Stride period | 0.4 s |
| Step height | 15 mm |
| Max step distance | 80 mm |
| Control loop rate | 50 Hz |

### Gait Phases

```
Phase A (cycle 0.0 – 0.5):  FL + RR in swing,  FR + RL in stance
Phase B (cycle 0.5 – 1.0):  FR + RL in swing,  FL + RR in stance
```

### Inverse Kinematics

Each leg uses analytic 2-link IK in the sagittal plane plus hip roll for lateral positioning:

1. **Hip roll** — computed from lateral (y) offset of foot target
2. **Hip pitch + knee** — 2-link planar IK (thigh + calf) for (x, z) foot target

### Velocity Handling

- Subscribes to `/cmd_vel` (`geometry_msgs/Twist`)
- Applies acceleration ramp limits for smooth velocity transitions
- Accounts for body angular velocity when computing per-leg foot targets
- Output: `/quadruped_joint_controller/commands` (`std_msgs/Float64MultiArray`, 12 values)

---

## Keyboard Controller

**File**: `src/quadruped_test_controller/quadruped_test_controller/keyboard_controller_node.py`

Reads raw terminal input from a background thread and publishes `geometry_msgs/Twist` to `/cmd_vel`.

| Key | Action | Speed |
|---|---|---|
| W | Forward | 0.3 m/s |
| S | Backward | 0.3 m/s |
| A | Strafe left | 0.2 m/s |
| D | Strafe right | 0.2 m/s |
| Q | Rotate CCW | 1.0 rad/s |
| E | Rotate CW | 1.0 rad/s |
| ESC / Ctrl+C | Stop & quit | — |

Requires `stdin_open: true` and `tty: true` in its `docker-compose.yaml`.

---

## Elevation Mapping

**Package**: `src/elevation_mapping_cupy_pkg/`  
**Upstream**: vendored `elevation_mapping_cupy` repo under `repo/`

Builds a 2.5D grid map of the terrain around the robot using GPU-accelerated point cloud processing (CuPy + PyTorch with CUDA 12.1).

| Parameter | Value |
|---|---|
| Map size | 3.0 m × 3.0 m |
| Resolution | 0.05 m (5 cm) |
| Update rate | 2 Hz |
| Map frame | `odom` |
| Sensor frame | `camera_link` |

Requires the NVIDIA Container Toolkit on the host for GPU passthrough.

---

## Nav2 MPPI Navigation

**Package**: `src/nav2_mppi_navigation/`  
**Launch**: `src/nav2_mppi_navigation/nav2_mppi_navigation/launch/nav2_mppi.launch.py`  
**Config**: `src/nav2_mppi_navigation/nav2_mppi_navigation/config/nav2_mppi_params.yaml`

Provides autonomous point-to-point navigation using the full Nav2 stack. Goals are sent via the RViz2 Nav2 panel ("2D Nav Goal"). No LIDAR or AMCL is required — the costmap reads the `/occupancy_grid` topic published by the elevation mapping container.

### Nav2 Node Graph

| Node | Role |
|---|---|
| `bt_navigator` | Executes Behaviour Tree; handles `navigate_to_pose` and `navigate_through_poses` actions |
| `planner_server` | Global path planning — NavFn (Dijkstra/A*) |
| `controller_server` | Local path tracking — **MPPI controller** at 20 Hz |
| `smoother_server` | Optional path smoothing (SimpleSmoother) |
| `behavior_server` | Recovery behaviours: spin, backup, drive_on_heading, wait |
| `waypoint_follower` | Multi-waypoint mission execution |
| `velocity_smoother` | Post-controller velocity ramping (open-loop, 20 Hz) |
| `collision_monitor` | Safety layer between velocity smoother and `/cmd_vel` (sources disabled by default) |

### MPPI Controller Parameters

| Parameter | Value |
|---|---|
| Plugin | `nav2_mppi_controller::MPPIController` |
| Controller frequency | 20 Hz |
| Time steps (horizon) | 56 steps × 0.05 s = 2.8 s |
| Batch size (rollouts) | 2 000 |
| Motion model | `DiffDrive` (change to `Omni` to enable lateral velocity) |
| Max forward velocity | 0.5 m/s |
| Max reverse velocity | 0.35 m/s |
| Max yaw rate | 1.9 rad/s |
| XY goal tolerance | 0.25 m |
| Yaw goal tolerance | 0.25 rad |
| Trajectory visualisation | enabled (markers published to RViz) |

**Active critics** (weighted cost terms sampled across rollouts):

| Critic | Weight | Purpose |
|---|---|---|
| `ConstraintCritic` | 4.0 | Enforce kinematic limits |
| `GoalCritic` | 5.0 | Drive toward goal position |
| `GoalAngleCritic` | 3.0 | Align heading at goal |
| `PathAlignCritic` | 14.0 | Stay aligned to planned path |
| `PathFollowCritic` | 5.0 | Track furthest reachable path point |
| `PathAngleCritic` | 2.0 | Penalise large heading deviations |
| `CostCritic` | 3.81 | Obstacle avoidance via costmap |
| `PreferForwardCritic` | 5.0 | Bias toward forward motion |

### Costmap Configuration

Both local and global costmaps are built from the live `/occupancy_grid` topic:

| Parameter | Local costmap | Global costmap |
|---|---|---|
| Frame | `odom` | `map` |
| Size | 4 m × 4 m (rolling) | full map |
| Resolution | 0.05 m | 0.05 m |
| Update rate | 5 Hz | 1 Hz |
| Plugins | `static_layer` + `inflation_layer` | `static_layer` + `inflation_layer` |
| Inflation radius | 0.55 m | 0.55 m |
| Robot footprint | 0.40 m × 0.30 m polygon | same |

### Localisation

A `static_transform_publisher` node broadcasts a fixed `map → odom` identity transform (fake localisation). This makes the map and odometry frames coincide, which is sufficient for simulation without AMCL or a laser scan.

### Tuning Notes

- To enable full omnidirectional navigation, set `motion_model: "Omni"` and `vy_max: 0.5` in `nav2_mppi_params.yaml`, and ensure the gait controller accepts non-zero lateral (`vy`) commands.
- `collision_monitor` is present but has `observation_sources: ""` (disabled). Add polygon or pointcloud sources there to add a hardware-level safety stop.
- Override frames or the occupancy topic at launch time via environment variables: `MAP_FRAME`, `ODOM_FRAME`, `BASE_FRAME`, `OCCUPANCY_GRID_TOPIC`.

---

## Key Environment Variables

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | DDS domain — change to isolate multiple ROS 2 systems |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS middleware implementation |
| `DISPLAY` | from host | X11 display for Gazebo GUI |
| `XAUTHORITY` | `/tmp/.docker.xauth` | X11 auth cookie |
| `NVIDIA_VISIBLE_DEVICES` | `all` | GPU visibility in CUDA containers |
| `NVIDIA_DRIVER_CAPABILITIES` | `all` | GPU capability flags |
| `MAP_FRAME` | `map` | Nav2 global frame |
| `ODOM_FRAME` | `odom` | Nav2 odometry frame |
| `BASE_FRAME` | `base_link` | Robot base frame used by Nav2 |
| `OCCUPANCY_GRID_TOPIC` | `/occupancy_grid` | Costmap source topic (from elevation mapping) |
| `USE_SIM_TIME` | `true` | Use `/clock` from Gazebo in Nav2 nodes |
