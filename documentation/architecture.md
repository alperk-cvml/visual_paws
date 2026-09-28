# Visual Paws — System Architecture

## Overview

Visual Paws is a modular ROS 2 quadruped navigation stack. It models a 12-DOF quadruped
(3 joints per leg × 4 legs) with **camera-only perception** — no LiDAR — where metric depth
is inferred from RGB by a neural network rather than measured. The stack runs against two
interchangeable front ends: Gazebo Harmonic (geometry, integration and planning logic) and
recorded video through the offline pipeline (the depth network, ORB-SLAM3 and real camera
behaviour, none of which Gazebo can validate).

Everything runs in Docker containers communicating over the host network via ROS 2 DDS.
The stack is currently in **Phase 2**: real depth inference and real localization, with the
Bézier-swing trot still driving the legs.

---

## Container Architecture

The root `docker-compose.yaml` uses Compose `include:` to aggregate independently-defined
service modules. Each module lives in its own subdirectory under `src/` with its own
`Dockerfile` and `docker-compose.yaml`.

All containers share:
- **Network**: `host` — zero-config ROS 2 DDS (CycloneDDS) discovery
- **IPC**: `host` — low-latency inter-process shared memory
- **Workspace**: repo root mounted at `/work/ws` (ROS 2 workspace)
- **ROS distribution**: Jazzy (Ubuntu 24.04)
- **User**: a non-root `USER` matching the host UID/GID. Not cosmetic — every container
  runs `colcon build` against the *same* bind-mounted workspace, so one root container
  poisons the shared `build/`/`install/`/`log/` directories for every other.

### Container Summary

| Container | Base Image | Role |
|---|---|---|
| `quadruped_gazebo` | `osrf/ros:jazzy-desktop-full` | Physics simulation, robot model, sensor data |
| `rviz` | `osrf/ros:jazzy-desktop-full` | Standalone visualisation (workspace mounted read-only) |
| `quadruped_gait_controller` | `osrf/ros:jazzy-desktop-full` | Spot Micro Bézier-swing gait (trot / walk) + whole-body IK |
| `quadruped_leg_odometry` | `osrf/ros:jazzy-desktop-full` | Leg kinematics + IMU → `/odom_lko` |
| `quadruped_test_controller` | `osrf/ros:jazzy-desktop-full` | Keyboard teleoperation |
| `monocular_depth` | `nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04` | Metric3D V2 → metric depth (production depth source) |
| `depth_comparison` | `nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04` | N depth backends on identical frames, one channel each |
| `depth_image_proc` | `osrf/ros:jazzy-desktop-full` | Depth image → point cloud conversion |
| `elevation_mapping_cupy` | `nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04` | GPU-accelerated 2.5D elevation mapping |
| `localization` | `ros:jazzy-ros-base` | ORB-SLAM3 + `robot_localization` EKF |
| `lidar_odometry` | `osrf/ros:jazzy-desktop-full` | KISS-ICP on any `PointCloud2` → `/odom_lidar` |
| `ground_plane_estimation` | `osrf/ros:jazzy-desktop-full` | RANSAC ground plane → depth scale probe (Alternative 1) |
| `odometry_scale_estimation` | `osrf/ros:jazzy-desktop-full` | Odometry chord comparison → depth scale probe (Alternative 2) |
| `pointcloud_scale_correction` | `osrf/ros:jazzy-desktop-full` | Applies a scale estimate back onto the cloud |
| `nav2_mppi_navigation` | `ros:jazzy-ros-base` | Nav2 stack with MPPI controller |
| `video_camera_sim` | `ros:jazzy-ros-base` | Recorded video → `/camera/image_raw` + `/camera/camera_info` |

> The images above are what the Dockerfiles currently build from. The Jetson deployment
> target uses the `dustynv/ros:jazzy-*-l4t-r36.4.0` equivalents; see CLAUDE.md.

### Compose entry points

The root compose file starts the Gazebo stack. Five narrower files select a subset of the
same module composes, each paired with its own env file:

| Entry point | Env file | Purpose |
|---|---|---|
| `docker-compose.yaml` | — / `offline-video.env` | Full Gazebo sim, or the offline video pipeline |
| `docker-compose-camera-pipeline.yaml` | `offline-video.env` | Offline video without Gazebo |
| `docker-compose-depth-comparison.yaml` | `depth-comparison.env` | Depth model comparison against Gazebo ground truth |
| `docker-compose-depth-scale-probe.yaml` | `depth-scale-probe.env` | Scale from ground-plane geometry (Alternative 1) |
| `docker-compose-odometry-scale-probe.yaml` | `odometry-scale-probe.env` | Scale from motion (Alternative 2) |
| `docker-compose-simulation-benchmark.yaml` | `simulation-benchmark.env` | Deterministic procedural Gazebo scenarios |

---

## ROS 2 Package Structure

```
src/
├── quadruped_gazebo/              # Simulation: URDF, ros2_control, benchmark scenarios
├── quadruped_kinematics/          # Pure-numpy Spot Micro geometry, leg IK/FK, whole-body IK
├── quadruped_gait_controller/     # Bézier-swing gait (gait.py) + ROS node
├── quadruped_leg_odometry/        # Leg kinematics + IMU odometry → /odom_lko
├── quadruped_test_controller/     # Keyboard teleop
├── video_camera_sim/              # Recorded video as camera topics
├── monocular_depth/               # Metric3D V2 inference node (production depth)
├── depth_comparison/              # Multi-backend depth comparison harness
├── dept_image_proc/               # Thin wrapper — depth → point cloud
├── elevation_mapping_cupy_pkg/    # GPU elevation map
│   ├── repo/                      # Vendored ANYbotics upstream — do not modify
│   └── simple_elevation_mapping_cupy/  # Wrapper package: launch + config
├── localization/                  # ORB-SLAM3 wrapper node + robot_localization EKF
│                                  # (ORB-SLAM3 itself is cloned into the image, not vendored)
├── lidar_odometry/                # KISS-ICP odometry from any PointCloud2
├── ground_plane_estimation/       # RANSAC ground plane → scale probe
├── odometry_scale_estimation/     # Odometry-chord scale probe
├── pointcloud_scale_correction/   # Applies a scale estimate to a cloud
├── nav2_mppi_navigation/          # Nav2 + MPPI
│   └── nav2_mppi_navigation/
│       ├── config/
│       │   ├── nav2_mppi_params.yaml  # Full Nav2 + MPPI parameter file
│       │   └── nav2.rviz              # RViz2 layout with Nav2 panel
│       └── launch/
│           └── nav2_mppi.launch.py    # Nav2 bringup (no TF of its own)
├── rviz/                          # Standalone visualisation container
├── elevation_mapping/             # Legacy (inactive)
└── mechdog_gazebo/                # Legacy URDF only (inactive)
```

`traversability_estimation` is described in the roadmap but is **not yet implemented** —
Nav2 currently consumes `/occupancy_grid` straight from elevation mapping, with no slope,
roughness or step-height cost applied.

---

## Robot Model

The robot is a **Spot Micro** (open-source KDY0532 / mike4192 geometry), defined in
`src/quadruped_gazebo/urdf/quadruped.urdf.xacro`.

- **Body**: 0.20 m × 0.09 m × 0.06 m chassis plus head and tail, 1.3 kg; 2.82 kg total
- **Hip pivots**: 0.186 m apart fore/aft, 0.078 m apart left/right
- **Legs**: 4 legs × 3 joints = 12 DOF total; all four knees point backward (Spot configuration)
- **Servos**: 3.4 N·m stall, 7 rad/s (35 kg·cm hobby-servo class), tracked in Gazebo by
  `gz_ros2_control`'s velocity loop (`position_proportional_gain` in `config/ros2_control.yaml`)

### Joint Layout (per leg)

```
body
 └── hip_roll    (abduction, about x; carries the 55 mm hip link outboard)
      └── hip_pitch  (about y; 107.5 mm upper leg)
           └── knee      (about y, always negative; 130 mm lower leg)
                └── foot  (15 mm rubber ball, fixed)
```

The same link lengths live in python in `src/quadruped_kinematics/` (`SPOT_MICRO`), the
package shared by the gait controller and leg odometry; a unit test there parses the xacro
and fails if the two diverge. The URDF derives its stand pose (`initial_value`) from the
geometry, so the robot spawns standing at `stand_height` and the gait controller's
`body_height` defaults to the same number.

### Sensors

| Sensor | Topic prefix | Purpose |
|---|---|---|
| Front RGB camera | `/front_camera/` | Input to the depth models in the comparison and scale profiles |
| Depth camera | `/depth_camera/` | Ground-truth 3D perception — sim only, no real counterpart |
| IMU | `/imu/data` | Orientation and angular rate for leg odometry and the EKF |

The depth camera has no hardware equivalent: on the real robot and in the offline pipeline
that role is filled by Metric3D V2 inference from the RGB image. This is why Gazebo cannot
validate the perception stack, only the geometry and planning built on top of it.

| Role | Gazebo | Real / offline |
|---|---|---|
| RGB image | `/front_camera/image_raw` | `/camera/image_raw` |
| Depth image | `/depth_camera/image_raw` (truth) | `/depth/image_raw` (inferred) |
| Camera info | `/depth_camera/camera_info` | `/camera/camera_info` |
| Point cloud | `/depth_camera/points` | `/depth/points` |

Nodes must take these as launch arguments; a hardcoded topic name breaks the sim/real
switch.

---

## Data Flow

The pipeline below the point cloud is identical in both modes. Only the *source* of the
depth image differs — and that difference is the whole point: Gazebo's depth is ground
truth, while the real pipeline's is inferred and carries a scale error that varies per
frame.

```
   ┌─── Gazebo sim ────────────┐      ┌─── Offline video ─────────┐
   │  quadruped physics        │      │  video_camera_sim         │
   │  /front_camera/*          │      │  /camera/image_raw        │
   │  /depth_camera/* (truth)  │      │  /camera/camera_info      │
   │  /joint_states, /imu/data │      └─────────────┬─────────────┘
   └───────────┬───────────────┘                    │
               │                       ┌────────────▼──────────────┐
               │                       │  monocular_depth          │
               │                       │  Metric3D V2, intrinsics- │
               │                       │  conditioned (fx = scale) │
               │                       └────────────┬──────────────┘
               │  /depth_camera/image_raw           │ /depth/image_raw
               └───────────────┬────────────────────┘
                               ▼
                    ┌────────────────────┐
                    │  depth_image_proc  │   cloud stays in camera_link_optical
                    │  depth → points    │   with the source image stamp
                    └─────────┬──────────┘
                              │ /depth_camera/points | /depth/points
                              ▼
                    ┌────────────────────────┐
                    │  elevation_mapping_cupy│  3 m × 3 m, 5 cm, 2 Hz
                    │  (GPU, CuPy)           │  frame: odom (sim) / map (offline)
                    └─────────┬──────────────┘
                              │ /elevation_map, /occupancy_grid
                              ▼
                    ┌──────────────────────────────┐
                    │  nav2_mppi_navigation        │
                    │  BT navigator → NavFn global │
                    │  → MPPI local (20 Hz)        │
                    └─────────┬────────────────────┘
                              │ /cmd_vel   (via velocity_smoother)                
                              ▼
┌────────────────────────┐   ┌────────────────────────┐
│  Keyboard Controller   │──▶│    Gait Controller     │
│  W/A/S/D/Q/E → Twist   │   │  /cmd_vel → IK → cmds  │
└────────────────────────┘   └──────────┬─────────────┘
 (manual override, same topic)          │ /quadruped_joint_controller/commands
                                        ▼
                              Gazebo joint actuators
```

Localization runs alongside, feeding `/tf` and `/odom` to everything above:

```
/joint_states + /imu/data ──▶ quadruped_leg_odometry ──▶ /odom_lko
/camera/image_raw + depth ──▶ ORB-SLAM3 (RGBD)      ──▶ map → odom
                                     └──▶ robot_localization EKF ──▶ /odom, odom → base_link
```

---

## Localization Modes

One `localization` container serves three modes, selected by `LOCALIZATION_MODE`. They
differ in who owns which TF edge — and since **each TF edge must have exactly one
publisher**, picking the wrong mode produces nondeterministic tf2 conflicts rather than an
error.

| Mode | `map → odom` | `odom → base_link` | Used by |
|---|---|---|---|
| `ground_truth` | static TF | Gazebo ground truth | Depth comparison, both scale probes |
| `slam` (default) | ORB-SLAM3 via EKF | `robot_localization` EKF | Full Gazebo sim |
| `offline_video` | ORB-SLAM3 owns `map → base_link` directly; no EKF | — | Offline video pipeline |

`ground_truth` exists because the comparison and probe profiles need a *TF tree*, not drift
correction — inserting an EKF there would blur the very quantity being measured.

---

## Perception: Depth

**Production node**: `src/monocular_depth/` — Metric3D V2, the only depth source in the
offline and real pipelines.

Metric3D takes its metric scale from the camera intrinsics, so the node subscribes to
CameraInfo and **drops frames until one arrives**. A wrong `fx` scales every depth value —
and the point cloud built from it — by the same factor, which is why `offline-video.env`
insists on the recording camera's real calibration.

Cost is the trade-off: roughly 1.2–1.5 s per frame at 640×480 on an RTX 2060, so
`target_fps` (default 10 Hz) is an upper bound the node will not reach. It drops what it
cannot keep up with, and elevation mapping receives clouds accordingly.

**Comparison harness**: `src/depth_comparison/` runs N backends on the *same* RGB frame and
publishes a depth image and cloud per model under `/depth_compare/<key>/`. Running them on
different frames would compare different moments and measure nothing. Backends: the Gazebo
depth sensor (relay reference), Depth Anything V2 (VKITTI and Hypersim checkpoints), and
Metric3D V2. A backend that fails to load is dropped with a warning; the node aborts only
if every channel fails.

Both images build from the repository root, because both need the shared `compat/mmcv/`
shim that stands in for Metric3D's mmcv 1.x imports.

---

## Perception: Monocular Depth Scale

Metric3D's output is metric *in principle*. In practice its scale drifts, and the two
modules below estimate that error from motion or geometry rather than from a ground-truth
depth image that a real robot would not have.

| Module | Method | Output |
|---|---|---|
| `ground_plane_estimation` | RANSAC ground plane; camera-to-plane distance ÷ expected camera height | `/ground_plane[/<key>]/{distance,camera_height,scale,marker}` |
| `odometry_scale_estimation` | Matched displacement chords between a metric odometry stream and a scale-bearing one, over a sliding window | `/odom_scale[/<key>]/{scale,shift,residual,ref_motion,marker}` |

Alternative 1 (`depth-scale-probe.env`) runs the ground-plane estimator with
`GROUND_PLANE_HEIGHT_REFERENCE_FRAME=odom`: under `ground_truth` localization, `odom`'s
z=0 plane *is* the real ground, so `/ground_plane/scale` becomes a live per-frame estimate
of the depth scale error.

Alternative 2 (`odometry-scale-probe.env`) feeds KISS-ICP (`lidar_odometry`) the *monocular*
cloud, so `/odom_lidar` inherits the depth's scale error, then compares it against
`/odom_lko` from leg kinematics. Chords are frame-invariant, so no extrinsic alignment is
needed. The profile also runs a second channel comparing Gazebo's true motion against the
same reference: it should sit at 1.0, and its deviation is the noise floor below which the
measurement means nothing.

Both publish raw per-window estimates. Smoothing, gating and staleness handling live in
`pointcloud_scale_correction`, which applies an estimate back onto the cloud and
republishes `<input>_corrected` with the original header untouched. Its error model is
pluggable: `scalar` (`p/s`) or `affine` (`(z−b)/s` along each ray, camera-optical frame
required).

See `documentation/ground_plane_estimation.md` and
`documentation/odometry_scale_estimation.md` for the derivations.

---

## Odometry Sources

| Module | Source | Topic | Failure mode |
|---|---|---|---|
| `quadruped_leg_odometry` | Leg kinematics (stance feet) + IMU | `/odom_lko` | Drift on slippery or uneven terrain |
| `localization` (ORB-SLAM3) | RGBD visual SLAM | `map → odom` | Texture-poor or dark scenes |
| `lidar_odometry` | KISS-ICP on any `PointCloud2` | `/odom_lidar` | Needs structure; no IMU fusion by design |

`lidar_odometry` is deliberately pure-LiDAR and sensor-agnostic: it is both the metric
reference for the scale work above and a ready odometry source for LiDAR-equipped platforms
under the vehicle interface contract. It publishes no TF by default — the EKF owns
`odom → base_link`. See `documentation/lidar_odometry.md` for the algorithm survey.

---

## Gait Controller

**Package**: `src/quadruped_gait_controller/` (`gait.py` pure python, `gait_node.py` ROS node)

Spot Micro Bézier-swing gait with whole-body inverse kinematics from `quadruped_kinematics`.

### Gaits (tuned defaults)

| `gait_type` | Cycle | Duty | Notes |
|---|---|---|---|
| `trot` (default) | 0.4 s | 0.70 | FL+RR / FR+RL pairs, 20 mm lift, four-foot support between 0.12 s swings |
| `walk` | 1.0 s | 0.75 | One foot in the air at a time (RL, FL, RR, FR); body shifted into the support triangle |

### Foot trajectories

- Stance: linear sweep from +Δ/2 to −Δ/2, Δ = body ground displacement over one
  stance period at the commanded velocity evaluated at that foot (`v + ω × r`).
- Swing: 11th-order Bézier (Hyun et al. 2014) from −Δ/2 to +Δ/2, tangent to the
  ground at both ends. The segments meet, so foot position is continuous.
- Whole-body IK: feet are planned in the nominal body frame; the body may be
  posed (height, roll, pitch, yaw, shift) relative to them.

### Velocity handling

- Subscribes to `/cmd_vel` (`geometry_msgs/Twist`), clamps (0.4 m/s forward, 0.25 m/s
  lateral, 1.2 rad/s yaw), ramps, and stops if no command arrives within 0.5 s
- `max_step_length` (0.10 m of foot travel per stance) is the real speed ceiling. When any
  leg's stride exceeds it, **every** leg is scaled by the same factor — which keeps a turn
  a turn rather than distorting its shape
- A zero command finishes the current cycle so all feet return to neutral, then holds the stand
- Output: `/quadruped_joint_controller/commands` (`std_msgs/Float64MultiArray`, 12 values, 100 Hz)

---

## Keyboard Controller

**File**: `src/quadruped_test_controller/quadruped_test_controller/keyboard_controller_node.py`

Reads raw terminal input from a background thread and publishes `geometry_msgs/Twist` to
`/cmd_vel`. Keys are held rather than latched: a key with no repeat within `HOLD_TIMEOUT`
(0.6 s) zeroes the command, and the node then publishes zero Twists for 0.5 s before going
silent — so releasing a key stops the robot without the node fighting Nav2 for `/cmd_vel`
once it is idle.

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
**Layout**: `repo/` holds the vendored ANYbotics `elevation_mapping_cupy` upstream — do not
modify it in place; change behaviour through parameters or through the wrapper package
`simple_elevation_mapping_cupy/`, which owns this module's launch files and config.

Builds a 2.5D grid map of the terrain around the robot using GPU-accelerated point cloud processing (CuPy + PyTorch with CUDA 12.1).

| Parameter | Env override | Value |
|---|---|---|
| Map size | `MAP_LENGTH_X` / `MAP_LENGTH_Y` | 3.0 m × 3.0 m |
| Resolution | `MAP_RESOLUTION` | 0.05 m (5 cm) |
| Update rate | `PUBLISH_RATE` | 2 Hz |
| Input cloud | `POINTCLOUD_TOPIC` | `/depth_camera/points` (sim) / `/depth/points` (real) |
| Map frame | config | `odom` in Gazebo, `map` in offline video |

The map frame differs by mode for a reason. Elevation mapping must accumulate in a
*smooth* frame, and in the dual-EKF sim the `map` frame jumps on loop closure, which would
corrupt the accumulated clouds. In offline video mode there is no EKF — ORB-SLAM3 owns
`map → base_link` directly — so `map` is the only continuous option available. It holds as
long as drift stays small; long sequences with loop closures can still break map coherence.

Requires the NVIDIA Container Toolkit on the host for GPU passthrough.

---

## Nav2 MPPI Navigation

**Package**: `src/nav2_mppi_navigation/`  
**Launch**: `src/nav2_mppi_navigation/nav2_mppi_navigation/launch/nav2_mppi.launch.py`  
**Config**: `src/nav2_mppi_navigation/nav2_mppi_navigation/config/nav2_mppi_params.yaml`

Provides autonomous point-to-point navigation using the full Nav2 stack. Goals are sent via the RViz2 Nav2 panel ("2D Nav Goal"). No LIDAR or AMCL is required — the costmap reads the `/occupancy_grid` topic published by the elevation mapping container.

That occupancy grid is raw height, not traversability: it is slope-unaware, and knows
nothing about roughness or step height against the leg's reach. The planned
`traversability_estimation` module that would supply `/traversability_costmap` does not
exist yet, so MPPI is currently avoiding obstacles by occupancy alone.

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
| `collision_monitor` | Configured in the params file but **not launched** — the bringup is included with `use_collision_monitor: False`, so `/cmd_vel` comes straight from the velocity smoother |

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

Nav2 runs without AMCL or a laser scan, and **the launch file publishes no TF of its own** —
`map → odom` must come from the `localization` container, in whichever mode the active
profile selects. (The launch file's docstring still claims a fake static `map → odom`; that
node was removed and the comment is stale.)

This is the correct arrangement, because each TF edge may have exactly one publisher: if
Nav2 also broadcast that edge it would conflict with the EKF whenever `slam` mode is up.

### Tuning Notes

- To enable full omnidirectional navigation, set `motion_model: "Omni"` and `vy_max: 0.5` in `nav2_mppi_params.yaml`, and ensure the gait controller accepts non-zero lateral (`vy`) commands.
- `collision_monitor` is configured with `observation_sources: ""` and is not launched at
  all. Enabling it means both adding polygon or pointcloud sources and flipping
  `use_collision_monitor` in the launch file.
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
| `USE_SIM_TIME` | `true` | Use `/clock` from Gazebo. **`false` in every offline-video profile** |
| `LOCALIZATION_MODE` | `slam` | `ground_truth` \| `slam` \| `offline_video` — see [Localization Modes](#localization-modes) |
| `GAIT_TYPE` | `trot` | `trot` \| `walk`; everything else is in `config/gait_params.yaml` |
| `POINTCLOUD_TOPIC` | `/depth_camera/points` | Cloud consumed by elevation mapping |
| `INPUT_IMAGE_TOPIC` | `/camera/image_raw` | RGB into Metric3D |
| `INPUT_CAMERA_INFO_TOPIC` | (alongside the image) | Intrinsics into Metric3D — **`fx` sets the metric scale** |
| `GROUND_PLANE_INPUT_TOPICS` | `/depth_camera/points` | Comma-separated; one ground-plane estimator per cloud |
| `SCALE_CORRECTION_INPUT_TOPICS` | `/depth_camera/points` | Comma-separated; one corrector node per cloud |
| `SCALE_CORRECTION_MODEL` | `scalar` | `scalar` (`p/s`) or `affine` (`(z−b)/s`) |
| `ODOM_SCALE_REFERENCE_TOPIC` | `/odom_lko` | The metric odometry the estimate is measured against |
| `ODOM_SCALE_ESTIMATE_TOPICS` | `/odom_lidar` | Comma-separated scale-bearing odometry channels |

Per-profile values live in the env files (`offline-video.env`, `depth-comparison.env`,
`depth-scale-probe.env`, `odometry-scale-probe.env`, `simulation-benchmark.env`), and each
is the single source of truth for its pipeline.
