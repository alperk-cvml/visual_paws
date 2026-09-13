# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Visual Paws** is a ROS 2 quadruped robot autonomous navigation stack. It targets real outdoor deployment on a 12-DOF quadruped (3 joints/leg × 4 legs) using **camera-only perception** (no LiDAR), running on NVIDIA Jetson Orin Nano / NX. The stack is designed for multi-vehicle extensibility (quadruped, Ackermann, tracked, USV) through a shared vehicle interface contract. Simulation runs in Gazebo Harmonic; deployment is fully containerized.

**Key stack**: ROS 2 Jazzy · Gazebo Harmonic · CycloneDDS · Python 3 / C++ · NVIDIA CUDA (GPU containers) · TensorRT (inference)

**Hardware target**: NVIDIA Jetson Orin Nano / NX (ARM64, JetPack 6.x, L4T r36.x)

---

## Design Principles

- **Camera-only perception**: No LiDAR. Metric depth is inferred from RGB via a neural network (Metric3D V2), conditioned on the camera intrinsics for metric scale.
- **Modular, not end-to-end**: Each layer is independently replaceable. AI is injected only at the locomotion layer (Phase 3 RL policy), not throughout the stack. This preserves debuggability, multi-vehicle reuse, and open-source reference value.
- **Vehicle-agnostic above the odometry boundary**: All platforms publish `nav_msgs/Odometry` + a traversability costmap. Elevation mapping, traversability, MPPI, and localization layers are shared across vehicle types. Only the odometry source and gait/drive controller differ per platform.
- **Docker-first**: Every module has its own `Dockerfile` and `docker-compose.yaml`. No bare-metal ROS installs.
- **Uncertainty is explicit**: Depth estimates carry per-pixel variance. This is an interface contract, not an optional extra.
- **Production hygiene from the start**: CI, unit tests, quantitative metrics, and honest trade-off documentation are built-in, not afterthoughts.

---

## Development Phases

| Phase | Milestone | Locomotion |
|---|---|---|
| **Phase 1** | Sim baseline: blind locomotion + elevation-map-aware routing | Fixed diagonal trot (analytic IK) |
| **Phase 2** | Traversability-aware routing, real depth inference, real localization | Fixed trot + terrain cost |
| **Phase 3** | Perceptive locomotion on real hardware; Isaac Lab RL policy | Perceptive RL policy (Isaac Lab → Jetson) |

> Current state: **Phase 2** — offline video pipeline operational (video replay → Metric3D V2 depth → ORB-SLAM3 localization → GPU elevation mapping). Gazebo sim pipeline also functional. Step 5 (elevation map accumulation from offline video) is the active work item.

---

## Build & Run

### Host Machine Setup (one-time for Gazebo GUI)
```bash
xhost +local:docker
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
```

### Start All Containers

**Gazebo simulation** (full sim pipeline):
```bash
docker compose up -d --build     # build images and start in background
./tmux-attach.sh                 # attach to quadruped_gazebo tmux session
./tmux-attach.sh --detach        # create all tmux sessions without attaching
```

**Offline video pipeline** (recorded video → depth estimation → SLAM → elevation map):
```bash
docker compose --env-file offline-video.env up -d --build
```

The `offline-video.env` profile sets `LOCALIZATION_MODE=offline_video` (ORB-SLAM3 directly owns `map→base_link`), `USE_SIM_TIME=false`, configures topic remappings, and adds a startup delay for model loading. See `offline-video.env` for all variables.

> **Note**: Place your video file at `videos/INPUT.MOV` (relative to repo root), or override `VIDEO_PATH` in `offline-video.env`.

**Depth model comparison** (Phase 1 — Gazebo ground truth vs. monocular models):
```bash
docker compose --env-file depth-comparison.env \
  -f docker-compose-depth-comparison.yaml up -d --build
```

The `depth-comparison.env` profile sets `USE_SIM_TIME=true`, `LOCALIZATION_MODE=ground_truth` (no EKF or ORB-SLAM3 — the comparison needs a TF tree, not drift correction), selects the deterministic `v0_geometry` simulation benchmark, and sets `COMPARE_BACKENDS=gt,dav2_vkitti,dav2_hypersim,metric3d_s`. `SIM_BENCHMARK_SEED=config` uses `random_seed: 42` from the scenario YAML. See `depth-comparison.env` for all variables.

**Depth scale probe** (Alternative 1 — live ground-plane scale estimate against the production Metric3D node):
```bash
docker compose --env-file depth-scale-probe.env \
  -f docker-compose-depth-scale-probe.yaml up -d --build
```

Runs `monocular_depth` (Metric3D V2, not the `depth_comparison` harness) on Gazebo's camera under `LOCALIZATION_MODE=ground_truth`, bridges its depth image to a point cloud via `depth_image_proc`, and feeds that to `ground_plane_estimation` with `GROUND_PLANE_HEIGHT_REFERENCE_FRAME=odom` — since `odom`'s z=0 plane is the true ground in this mode, `/ground_plane/scale` becomes a live per-frame estimate of Metric3D's depth scale error, no ground-truth depth image required. See `documentation/ground_plane_estimation.md` for the derivation.

### Inside Containers (ROS 2 Workspace)
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install                              # build all packages
colcon build --symlink-install --packages-select <pkg>     # build one package
source install/setup.bash
ros2 launch <pkg> <launch_file.py>
```

### Attach to Keyboard Controller
```bash
tmux attach -t quadruped_test_controller   # W/A/S/D forward/strafe, Q/E rotate, ESC quit
```

### Monitor ROS Topics
```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

---


## ROS 2 Conventions & TF Standards
 
All code must follow **REP-105** (coordinate frames) and **REP-103** (units/axes). Non-compliant frame names or tree structures are bugs, not style issues.
 
### Coordinate Conventions (REP-103)
- x forward, y left, z up (right-hand rule)
- Angles in radians, right-hand rule about z (counter-clockwise positive)
- Units: metres, seconds, radians throughout. No millimetres, degrees, or mixed units.
### Canonical Frame Names (REP-105)
 
| Frame | Meaning |
|---|---|
| `map` | Fixed world frame; origin set at ORB-SLAM3 init |
| `odom` | Continuous odometry frame; no discrete jumps |
| `base_link` | Robot body centre (rigid body) |
| `base_footprint` | Projection of `base_link` onto ground plane |
| `camera_link` | Camera rigid body origin |
| `camera_optical` | Camera optical frame (z forward, x right, y down) |
| `imu_link` | IMU sensor frame |
 
**Never invent frame names** (`robot`, `world`, `camera_frame`, `base`, `map_frame`). Use the names above. New sensor frames follow `<sensor>_link` (rigid body) and `<sensor>_optical` (optical frame).
 
### Authoritative TF Tree
 
```
map
└── odom                     (publisher: robot_localization EKF; Phase 1: static TF)
    └── base_link            (publisher: robot_localization EKF / ros2_control)
        ├── base_footprint   (publisher: robot_state_publisher, static)
        ├── imu_link         (publisher: robot_state_publisher, static)
        ├── camera_link      (publisher: robot_state_publisher, static)
        │   └── camera_optical (publisher: robot_state_publisher, static)
        └── [leg frames]     (publisher: robot_state_publisher, from joint states)
```
 
Rules:
- **Each TF edge has exactly one publisher.** Adding a publisher means removing the old one. Duplicate publishers cause nondeterministic tf2 conflicts.
- ORB-SLAM3 publishes **`map→odom`**, not `map→base_link`. It corrects drift via the odom frame, leaving `odom→base_link` to odometry/EKF.
- Static transforms come from `robot_state_publisher` or `static_transform_publisher` — never a dynamic publisher emitting fixed values.
- `elevation_mapping_cupy` fixed frame is **`map`** in offline-video mode (ORB-SLAM3 owns `map→base_link` directly). In Gazebo sim with dual-EKF, use **`odom`** — elevation mapping must run in a *smooth* frame; `map` frame discontinuities from loop closure would corrupt accumulated point clouds.
- **Point clouds from `depth_image_proc` are in `camera_link_optical` and carry the source image timestamp. Never restamp or reframe them.** This is the single most common Step 4 failure.
---
 
## Offline Video Workflow
 
The offline pipeline processes pre-recorded video files through the full perception-to-mapping stack without Gazebo or live sensors.
 
### Pipeline
```
video file → video_camera_sim → /camera/image_raw
                                    ↓
                        monocular_depth (Metric3D V2) → /depth/image_raw
                                    ↓
                              depth_image_proc → /depth_camera/points
                                    ↓
          ORB-SLAM3 (localization) ← /camera/image_raw
              ↓ TF: map→base_link
          elevation_mapping_cupy ← /depth_camera/points + TF
              ↓
          /elevation_map (GridMap)
```

### Launch
```bash
docker compose --env-file offline-video.env up -d --build
```

### Key differences from Gazebo sim
| Aspect | Gazebo sim | Offline video |
|---|---|---|
| `USE_SIM_TIME` | `true` | `false` |
| `LOCALIZATION_MODE` | `slam` (dual-EKF) | `offline_video` (ORB-SLAM owns `map→base_link`) |
| TF for `base_link→camera_link` | `robot_state_publisher` | Static TF from launch file |
| Depth source | Ground-truth depth camera | Metric3D V2 (monocular inference) |
| Elevation map frame | `odom` | `map` |

### Rules
- **`offline-video.env` is the single source of truth** for the offline pipeline. All topic names, frame IDs, and mode switches are set there.
- **`VIDEO_START_DELAY`** (default 20s) gives ORB-SLAM3 vocabulary and the Metric3D V2 model time to load before video playback starts. Reduce only if models are pre-cached.
- **`VIDEO_LOOP=false`** — looping resets image motion while retaining the SLAM map, corrupting localization.
- Every node runs with `use_sim_time: false` — wall-clock timestamps are used throughout.
---
 
## Sim vs. Real Topic Names
 
The same pipeline runs against two sources. Do not hardcode either.
 
| Role | Gazebo (Phase 1) | Real / bag (Phase 2+) |
|---|---|---|
| RGB image | `/front_camera/image_raw` | `/camera/image_raw` |
| Depth image | `/depth_camera/image_raw` (ground truth) | `/depth/image_raw` (Metric3D V2 inferred) |
| Camera info | `/depth_camera/camera_info` | `/camera/camera_info` |
| Point cloud | `/depth_camera/points` | `/depth/points` |
 
All topic names must be remappable via launch arguments. Nodes that hardcode topic names break the sim/real switch.
 
> **Critical**: Gazebo depth is ground truth. Real depth is *inferred* and carries
> scale/shift error that varies per frame. **Gazebo cannot validate the perception
> stack.** Geometry, integration, and planning logic are testable in sim; the depth network,
> ORB-SLAM3, and real camera behaviour (auto-exposure, rolling shutter, motion
> blur) require recorded footage via the offline video pipeline as a separate test bench.
 
---


## Architecture

### Full Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│  PERCEPTION                                                     │
│                                                                 │
│  RGB Camera + CameraInfo ──→ Metric3D V2 ──→ /depth/image_raw  │
│                          (normals head not yet wired:           │
│                           /depth/surface_normals is planned)    │
│                                                                 │
│  /depth/image_raw + /camera/camera_info                        │
│    └──→ depth_image_proc ──→ /depth/points (PointCloud2)       │
└─────────────────────────────────────────────────────────────────┘
                          │ /depth/points
                          │ /depth/surface_normals
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│  MAPPING                                                        │
│                                                                 │
│  elevation_mapping_cupy                                         │
│    ← /depth/points (covariance-weighted fusion, GPU)           │
│    ← /depth/surface_normals (slope/roughness inputs)           │
│    ← /odom (robot pose)                                        │
│    → /elevation_map  (grid_map)                                │
│                                                                 │
│  traversability_estimation                                      │
│    ← /elevation_map                                            │
│    → /traversability_map  (slope + roughness + step height)    │
│    → /traversability_costmap  (nav_msgs/OccupancyGrid)         │
└─────────────────────────────────────────────────────────────────┘
                          │ /traversability_costmap
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│  LOCALIZATION                          (Phase 2+)               │
│                                                                 │
│  Gait Controller ──→ /odom_lko   (leg kinematics odometry)     │
│  ORB-SLAM3 (RGBD) ──→ /odom_slam (loop-closure drift corr.)    │
│                                                                 │
│  robot_localization EKF                                        │
│    ← /odom_lko + /odom_slam + /imu/data                       │
│    → /odom  (fused, authoritative)                             │
│    → /tf: map → odom                                           │
│                                                                 │
│  Phase 1 fallback: static map→odom TF (fake localisation)      │
└─────────────────────────────────────────────────────────────────┘
                          │ /odom, /tf
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│  PLANNING & CONTROL                                             │
│                                                                 │
│  Nav2 (SmacPlannerLattice global, MPPI local)                  │
│    ← /traversability_costmap                                   │
│    ← /odom, /tf                                                │
│    → /cmd_vel  (Twist: linear.x/y + angular.z)                │
│    goal: /navigate_to_pose action (RViz2 or QGC mission)       │
└─────────────────────────────────────────────────────────────────┘
                          │ /cmd_vel
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│  LOCOMOTION                                                     │
│                                                                 │
│  Phase 1–2: Fixed diagonal trot (analytic IK, 50 Hz)           │
│  Phase 3:   Perceptive RL policy (Isaac Lab → TensorRT/Jetson) │
│               ← /cmd_vel + /elevation_map (proprioception)     │
│               → /quadruped_joint_controller/commands           │
└─────────────────────────────────────────────────────────────────┘
```

### Vehicle Interface Contract (Multi-Vehicle Boundary)

Everything above the following interface is vehicle-agnostic and shared across platforms:

| Topic / TF | Type | Provider per vehicle |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | LKO (quadruped), wheel odom (Ackermann/tracked), GPS+DVL (USV) |
| `/traversability_costmap` | `nav_msgs/OccupancyGrid` | Shared (elevation_mapping_cupy + traversability_estimation) |
| `/tf: base_link → odom` | TF2 | From odometry source above |
| `/imu/data` | `sensor_msgs/Imu` | Platform IMU |

The MPPI controller, traversability estimator, elevation mapper, and localization EKF consume only these interfaces and are never coupled to a specific vehicle.

### Container Layout

The root `docker-compose.yaml` uses Compose `include:` to pull in each module's own `docker-compose.yaml`. All containers run on the host network with `ipc: host` for zero-config ROS 2 DDS discovery.

Two narrower entry points select a subset of the same module composes: `docker-compose-camera-pipeline.yaml` (offline video, no Gazebo) and `docker-compose-depth-comparison.yaml` (Phase 1 depth model comparison). Each pairs with its own env file — see [Environment Variables](#environment-variables-set-in-docker-compose).

| Container | Image Base | Phase | Role |
|---|---|---|---|
| `quadruped_gazebo` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 1+ | Physics sim, robot spawn, sensor publishing |
| `rviz` | `osrf/ros:jazzy-desktop-full` | 1+ | Standalone visualization for simulation and offline pipelines |
| `monocular_depth` | `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` | 2+ | Metric3D V2 → metric depth |
| `depth_comparison` | `nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04` | 2+ | N depth models on identical frames → depth + cloud per model |
| `depth_image_proc` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 1+ | Depth → point cloud conversion |
| `elevation_mapping_cupy` | `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` | 1+ | GPU elevation mapping |
| `traversability_estimation` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 2+ | Slope/roughness/step costmap from elevation map |
| `localization` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 2+ | ORB-SLAM3 + LKO → robot_localization EKF |
| `lidar_odometry` | `osrf/ros:jazzy-desktop-full` | 2+ | KISS-ICP LiDAR odometry → `/odom_lidar` (metric reference / multi-vehicle odom source) |
| `nav2_mppi_navigation` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 1+ | Nav2 + MPPI, traversability costmap consumer |
| `quadruped_gait_controller` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 1–2 | Analytic IK + diagonal trot; publishes LKO |
| `quadruped_rl_controller` | `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` | 3 | TensorRT perceptive RL policy from Isaac Lab |
| `quadruped_test_controller` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 1+ | Keyboard teleop |
| `video_camera_sim` | `ros:jazzy-ros-base` | 2+ | Plays pre-recorded video as camera topics for offline testing |

> **Base image note**: Use `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` for CPU/ROS-only nodes. Use `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` for nodes requiring GPU inference (monocular depth, elevation mapping, RL policy). These images ship pre-built PyTorch + CUDA for Jetson; avoid `nvidia/cuda` + manual PyTorch installs on ARM64.

### ROS 2 Topic Flow (complete)

```
[Phase 1 – Sim only]
Gazebo Sim
  → /tf, /imu/data, /front_camera/*, /depth_camera/*

Depth Image Proc
  ← /depth_camera/image_raw + /depth_camera/camera_info
  → /depth_camera/points

Elevation Mapping CuPy
  ← /depth_camera/points
  → /elevation_map, /occupancy_grid  (raw, slope-unaware)

Nav2 MPPI Navigation
  ← /occupancy_grid, /odom (fake: static map→odom TF), /tf
  → /cmd_vel

Gait Controller
  ← /cmd_vel
  → /quadruped_joint_controller/commands

─────────────────────────────────────────────────────────────────

[Phase 2+ – Real hardware / full stack]
RGB Camera
  → /camera/image_raw, /camera/camera_info

Video Camera Sim (offline testing, replaces live camera)
  ← pre-recorded video file
  → /camera/image_raw     (sensor_msgs/Image, BGR8)
  → /camera/camera_info   (sensor_msgs/CameraInfo)

Monocular Depth (Metric3D V2)
  ← /camera/image_raw + /camera/camera_info  (intrinsics = metric scale)
  → /depth/image_raw      (metric, in metres)
  → /depth/surface_normals (planned; Metric3D's normal head is not wired yet)

Depth Image Proc
  ← /depth/image_raw + /camera/camera_info
  → /depth/points  (PointCloud2)

Elevation Mapping CuPy
  ← /depth/points
  → /elevation_map  (grid_map_msgs/GridMap)

Traversability Estimation
  ← /elevation_map
  → /traversability_map
  → /traversability_costmap  (nav_msgs/OccupancyGrid)

Localization
  ← /odom_lko (from gait controller joint states + kinematics)
  ← /camera/image_raw + /depth/image_raw  (ORB-SLAM3 RGBD)
  ← /imu/data
  → /odom  (robot_localization EKF fusion)
  → /tf: map→odom

Nav2 MPPI Navigation
  ← /traversability_costmap
  ← /odom, /tf
  → /cmd_vel

Gait Controller (Phase 1–2) / RL Policy (Phase 3)
  ← /cmd_vel
  ← /elevation_map  (Phase 3 only: foothold-aware)
  → /quadruped_joint_controller/commands
  → /odom_lko  (leg kinematics odometry, published by gait controller)
```

---

## Key Source Files

- `src/quadruped_gait_controller/quadruped_gait_controller/gait_node.py` — Diagonal trot gait with analytic 3-DOF IK. Publishes `/odom_lko` (leg kinematics odometry) in addition to joint commands. Phase A (0–0.5): FL+RR swing; Phase B: FR+RL swing. 50 Hz, 0.4s stride, 15mm step height.
- `src/quadruped_gazebo/urdf/quadruped.urdf.xacro` — Robot URDF: 12 joints (hip_roll, hip_pitch, knee per leg), depth camera, IMU, RGB camera.
- `src/quadruped_gazebo/config/ros2_control.yaml` — JointGroupPositionController mapping for all 12 joints.
- `src/quadruped_gazebo/launch/gazebo.launch.py` — Spawns robot, loads controllers, starts ROS-Gazebo bridges.
- `src/video_camera_sim/` — Plays pre-recorded video files as `/camera/image_raw` + `/camera/camera_info` for offline testing of perception and localization pipelines. Configurable FPS, loop, and camera intrinsics.
- `src/monocular_depth/` — (Phase 2) Metric3D V2 inference node — the production depth source. Intrinsics-conditioned, so it needs CameraInfo and drops frames until one arrives. TensorRT FP16 export for Jetson is still to come.
- `src/depth_comparison/` — Runs N depth backends on the *same* RGB frame and publishes one metric depth image + point cloud per model under `/depth_compare/<key>/`. Backends: Gazebo depth sensor (relay reference), Depth Anything V2 (VKITTI + Hypersim checkpoints), Metric3D V2. Add a model by implementing `DepthBackend` and registering it in `backends/__init__.py`. Shares the repo-root `compat/mmcv/` shim (a deliberate mmengine-backed stand-in for Metric3D's mmcv 1.x imports) with `monocular_depth`; both images therefore build from the repository root, not their own directory.
- `src/lidar_odometry/` — KISS-ICP LiDAR odometry from any `PointCloud2`; publishes `/odom_lidar` (`odom`→`base_link`, twist in `base_link`, drift-model covariance). Pure-lidar on purpose (sensor-agnostic, no IMU sync): the metric reference for the monocular scale work (Alternative 2) and an odometry source for lidar-equipped platforms of the vehicle contract. No TF by default (EKF owns `odom→base_link`). See `documentation/lidar_odometry.md` for the full algorithm survey (KISS-ICP vs. FAST-LIO2/Point-LIO/DLIO/LIO-SAM) and rationale.
- `src/ground_plane_estimation/` — RANSAC ground plane from any depth point cloud; publishes `/ground_plane[/<key>]/{distance,camera_height,scale,marker}`. The kinetic scale-hypothesis probe: with a height reference (`odom` z=0 in ground-truth sim, or a nominal standing height), `scale` = camera-to-plane distance / expected camera height ≈ per-frame monocular depth scale factor. One estimator per input topic; pairs with the depth comparison channels via `GROUND_PLANE_INPUT_TOPICS`.
- `src/elevation_mapping_cupy_pkg/` — GPU elevation map (3m×3m, 5cm resolution, 2 Hz). Ingests point cloud + surface normals for covariance-weighted fusion. Vendored `elevation_mapping_cupy` repo.
- `src/traversability_estimation/` — (Phase 2) Computes traversability costmap from elevation map: slope angle, roughness (height variance), step height against leg reach limits. Publishes `nav_msgs/OccupancyGrid`.
- `src/localization/` — (Phase 2) ORB-SLAM3 in RGBD mode (monocular depth as D channel) + `robot_localization` EKF fusing LKO + SLAM + IMU. LKO is the primary short-horizon source; SLAM provides loop-closure drift correction.
- `src/nav2_mppi_navigation/nav2_mppi_navigation/config/nav2_mppi_params.yaml` — Nav2 + MPPI config. MPPI: 2000 rollouts, 56-step horizon, `Omni` motion model (quadruped is holonomic). Costmap reads `/traversability_costmap`, 4m×4m rolling window, 5cm resolution.
- `src/nav2_mppi_navigation/nav2_mppi_navigation/launch/nav2_mppi.launch.py` — Phase 1: static `map→odom` TF. Phase 2+: consumes `/odom` + `/tf` from localization container.

## Vendored Code — Do Not Modify

These directories contain third-party code vendored into the repo. Changes
here are lost on update and make upstream fixes unmergeable.

- `src/elevation_mapping_cupy_pkg/elevation_mapping_cupy/` — upstream ANYbotics repo
- `src/localization/ORB_SLAM3/` — upstream ORB-SLAM3

To change behaviour, use configuration/parameters, a wrapper node, or a
documented patch file in `patches/`. Never edit vendored sources in place.
If a change genuinely requires modifying upstream, say so explicitly and
propose it as a patch, do not do it silently.

---

## Localization Strategy

| Source | Role | Failure mode |
|---|---|---|
| Leg kinematics odometry (LKO) | Primary short-horizon odometry | Drift on slippery/uneven terrain |
| ORB-SLAM3 (RGBD, monocular depth as D) | Loop closure, long-horizon drift correction | Fails on texture-poor / dark environments |
| IMU | Angular velocity + gravity vector | No position, biases over time |
| `robot_localization` EKF | Fuses all three → `/odom` | Single point of failure; tune noise covariances carefully |

LKO is the fallback when vision degrades. SLAM loop closure corrects accumulated LKO drift. Neither is used alone.

---

## Locomotion Strategy

| Phase | Approach | Tradeoffs |
|---|---|---|
| 1–2 | Fixed diagonal trot, analytic IK | Simple, reliable on flat/mildly uneven terrain; no terrain adaptation |
| 3 | Perceptive RL policy (Isaac Lab → TensorRT) | Graceful degradation on noisy monocular depth; zero-shot Jetson Orin transfer via NVIDIA-documented path; requires Isaac Lab training infrastructure |

**Why RL over OCS2 MPC for Phase 3**: MPC-based foothold planning (e.g. `legged_perceptive`) requires accurate terrain geometry — sensitive to depth noise from monocular estimation. RL policies trained with domain randomization in Isaac Lab are more robust to noisy inputs, and NVIDIA provides a documented sim-to-real transfer path directly to Jetson Orin hardware.

---

## Multi-Vehicle Extensibility

To add a new platform, implement the vehicle interface contract (see above) and provide a platform-specific odometry source and drive/gait controller. Nothing above the `/odom` + `/traversability_costmap` boundary changes.

| Platform | Odometry source | Drive controller |
|---|---|---|
| Quadruped | LKO + ORB-SLAM3 EKF | Gait controller (trot / RL policy) |
| Ackermann | Wheel encoder + GPS | `ackermann_steering_controller` (ros2_control) |
| Tracked | Wheel encoder + IMU | Differential drive controller |
| USV | GPS + DVL | Thruster controller |

---

## Adding a New Module

1. Create `src/<module>/` with `package.xml`, `docker-compose.yaml`, and either `CMakeLists.txt` (C++) or `setup.py` (Python).
2. Choose base image: `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` (CPU) or `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` (GPU/inference).
3. Add `include: - path: src/<module>/docker-compose.yaml` to root `docker-compose.yaml`.
4. Add the container name to the `CONTAINERS` array in `tmux-attach.sh`.
5. Add at least one launch smoke test (node starts, required topics appear within 10s).

---

## Environment Variables (set in docker-compose)

| Variable | Default | Purpose |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | Isolate ROS 2 DDS domains |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS middleware |
| `DISPLAY` / `XAUTHORITY` | from host | Gazebo, RViz, and ORB-SLAM3 viewer GUIs via X11 |
| `NVIDIA_VISIBLE_DEVICES` | `all` | GPU access in CUDA/PyTorch containers |
| `CUDA_VISIBLE_DEVICES` | `0` | Single GPU, avoids allocation conflicts |

### Offline Video Profile (`offline-video.env`)

| Variable | Value | Purpose |
|---|---|---|
| `USE_SIM_TIME` | `false` | Wall-clock timestamps (no `/clock` topic) |
| `LOCALIZATION_MODE` | `offline_video` | ORB-SLAM3 owns `map→base_link` directly |
| `MODE` | `RGBD` | ORB-SLAM3 tracking mode |
| `ENABLE_VIEWER` | `true` | ORB-SLAM3 Pangolin viewer |
| `VIDEO_PATH` | `/work/ws/videos/INPUT.MOV` | Input video file path |
| `VIDEO_LOOP` | `false` | Single playthrough (loop corrupts SLAM) |
| `VIDEO_FPS` | `10.0` | Playback frame rate |
| `VIDEO_START_DELAY` | `20.0` | Seconds to wait for model loading |
| `POINTCLOUD_TOPIC` | `/depth_camera/points` | Point cloud topic for elevation mapping |
| `MAP_FRAME_ID` | `map` | Elevation map fixed frame |
| `FRAME_ID` | `camera_link_optical` | Camera frame for video publisher |
| `INPUT_CAMERA_INFO_TOPIC` | `/camera/camera_info` | Intrinsics for Metric3D; empty resolves alongside the image topic |
| `FX` / `FY` / `CX` / `CY` | video_camera_sim defaults | **Set these to the recording camera's real calibration.** Metric3D derives metric scale from `fx`, so a wrong value scales all depth (and the point cloud) by the same factor |

### Depth Comparison Profile (`depth-comparison.env`)

Used with `docker-compose-depth-comparison.yaml` (Phase 1, Gazebo).

| Variable | Value | Purpose |
|---|---|---|
| `USE_SIM_TIME` | `true` | Gazebo publishes `/clock` |
| `LOCALIZATION_MODE` | `ground_truth` | Static `map→odom` + `odom→base_link`; no EKF or ORB-SLAM3 |
| `COMPARE_INPUT_IMAGE_TOPIC` | `/front_camera/image_raw` | RGB source every model shares |
| `COMPARE_INPUT_CAMERA_INFO_TOPIC` | `/front_camera/camera_info` | Intrinsics (Metric3D needs these for metric scale) |
| `COMPARE_BACKENDS` | `gt,dav2_vkitti,dav2_hypersim,metric3d_s` | Channels to run; drop entries to keep the loop responsive |
| `COMPARE_NAMESPACE` | `/depth_compare` | Parent of `<key>/image_raw`, `<key>/camera_info`, `<key>/points` |
| `COMPARE_TARGET_FPS` | `1.0` | Upper bound only — all models run per frame |
| `COMPARE_PUBLISH_POINTCLOUDS` | `true` | Spawn a `point_cloud_xyz_node` per channel |
| `SIM_BENCHMARK_SCENARIO` | `v0_geometry` | Procedural Gazebo scenario |
| `SIM_BENCHMARK_SEED` | `config` | Uses the scenario YAML seed (`42`); an integer overrides it |
| `SIM_BENCHMARK_OUTPUT_DIR` | `/work/ws/build/simulation_benchmark` | Generated benchmark artifact root |

Rules:
- **All models run on the same frame.** Rate-limiting separate nodes would let them drift onto different source frames, and comparing depth maps of different moments measures nothing.
- **`max_depth` is not a free parameter** for Depth Anything V2. Its metric head ends in `sigmoid(x) * max_depth`, so the value is a trained-in output scale — raising Hypersim's 20 m to cover an outdoor scene multiplies every prediction by 4.
- **A backend that fails to load is dropped, not fatal.** The node aborts only if every channel fails. Check startup logs for `unavailable, dropping it` before trusting a comparison.
- Metric3D reaches ~1167 ms/frame on an RTX 2060 against ~300 ms for the DA-V2 models; all three together run at roughly 0.55 Hz.
- The same cost applies to the production `monocular_depth` node now that it runs Metric3D: measured ~1.2–1.5 s/frame at 640×480 on this host, so its `target_fps` (default 10 Hz) is an upper bound it will not reach — the node simply drops what it cannot keep up with. Expect roughly 4× fewer point clouds reaching elevation mapping than under DA-V2, and budget accordingly on Orin.

---

## Known Limitations & Open Decisions

- **Stereo trade-off**: NVIDIA Isaac ROS (`isaac_ros_dnn_stereo_depth`, cuVSLAM, FoundationStereo) is significantly richer for stereo pairs. If a second camera is affordable, stereo should be reconsidered before committing to monocular-only.
- **MPPI motion model**: Currently `Omni` for holonomic quadruped motion. Tune `vy_max` to match actual lateral gait speed.
- **Traversability costmap resolution**: 5cm grid at 3m×3m is appropriate for indoor/controlled outdoor. Expand to 10m×10m for open terrain but watch GPU memory on Orin Nano.
- **ORB-SLAM3 monocular scale**: In `MONOCULAR` mode, ORB-SLAM3 scale is metric-consistent only if the monocular depth output is metric-accurate. In `RGBD` mode with monocular depth as the D channel, scale comes from the depth network. Validate carefully on real hardware.
- **Elevation map frame in offline mode**: Using `map` frame works when ORB-SLAM3 drift is small. For longer sequences with loop closures, the map frame may have discontinuities that affect elevation map coherence.
- **QGroundControl integration**: Planned. Requires MAVLink bridge (`MAVROS` or `px4_ros_com`) publishing mission waypoints as `nav2_msgs/action/NavigateThroughPoses`.




## Current Target

Now the pipeline is working. But Depth Anything V2 has a huge scale problem. I want to implement other models and compare them side to side. I'll first try comparing with visual inspection, if this is not enough I'll develop a quantitative comparison module.

- [x] Step 0: DepthAnything V2.
- [x] Step 00: ROS Depth Sensor module
- [X] Step 1: DAV2 VKITTI
- [X] Step 2: DAV2 Hypersim
- [X] Step 3: Metric3D-v2-S

I'll thest these in two phases:
- [] Phase 1: ROS2 Gazebo <-- We are here
- [] Phase 2: On pre-recorded (iphone) videos

Seems like Metric3D is currently the closest alternative.

We will take a detour. I'll try if I can solve the depth problem kinetically. I have two alternatives
 - [ ] Alternative 1: Solve it via ground plane distance
 - [ ] Alternative 2: Use IMU and leg odometry to estimate the motion and compare it to the motion of inferred with monodepth pointcloud.