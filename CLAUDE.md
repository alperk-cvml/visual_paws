# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Visual Paws** is a ROS 2 quadruped robot autonomous navigation stack. It targets real outdoor deployment on a 12-DOF quadruped (3 joints/leg × 4 legs) using **camera-only perception** (no LiDAR) from a **stereo camera — Stereolabs ZED 2i, 2.1 mm lens** — running on NVIDIA Jetson Orin Nano / NX. The earlier monocular-depth path is on hold (see [Current Target](#current-target)). The stack is designed for multi-vehicle extensibility (quadruped, Ackermann, tracked, USV) through a shared vehicle interface contract. Simulation runs in Gazebo Harmonic; deployment is fully containerized.

**Key stack**: ROS 2 Jazzy · Gazebo Harmonic · CycloneDDS · Python 3 / C++ · NVIDIA CUDA (GPU containers) · TensorRT (inference) · ZED SDK + zed-ros2-wrapper (stereo depth, planned)

**Hardware target**: NVIDIA Jetson Orin Nano / NX (ARM64, JetPack 6.x, L4T r36.x)

---

## Design Principles

- **Camera-only perception, stereo depth**: No LiDAR. Metric depth comes from stereo disparity on a ZED 2i (12 cm baseline, 2.1 mm lens), computed by the ZED SDK on hardware: scale is fixed by the calibrated baseline, not learned. The ZED SDK cannot ingest Gazebo images, so in simulation depth comes from ground truth plus a ZED-like error model (see [Current Target](#current-target)). The monocular path (Metric3D V2, intrinsics-conditioned) is **on hold** because of its per-frame scale error. Its modules stay in the repo and must keep building, but they are no longer the production depth source.
- **Modular, not end-to-end**: Each layer is independently replaceable. AI is injected only at the locomotion layer (Phase 3 RL policy), not throughout the stack. This preserves debuggability, multi-vehicle reuse, and open-source reference value.
- **Vehicle-agnostic above the odometry boundary**: All platforms publish `nav_msgs/Odometry` + a traversability costmap. Elevation mapping, traversability, MPPI, and localization layers are shared across vehicle types. Only the odometry source and gait/drive controller differ per platform.
- **Docker-first**: Every module has its own `Dockerfile` and `docker-compose.yaml`. No bare-metal ROS installs.
- **Uncertainty is explicit**: Depth estimates carry per-pixel variance. This is an interface contract, not an optional extra. For stereo, derive it from the matcher's confidence and the disparity noise model σ_Z ≈ Z²·σ_d / (f·B): error grows with the square of range.
- **Production hygiene from the start**: CI, unit tests, quantitative metrics, and honest trade-off documentation are built-in, not afterthoughts.

---

## Development Phases

| Phase | Milestone | Locomotion |
|---|---|---|
| **Phase 1** | Sim baseline: blind locomotion + elevation-map-aware routing | Bézier-swing trot (analytic IK, Spot Micro) |
| **Phase 2** | Traversability-aware routing, real depth inference, real localization | Fixed trot + terrain cost |
| **Phase 3** | Perceptive locomotion on real hardware; Isaac Lab RL policy | Perceptive RL policy (Isaac Lab → Jetson) |

> Current state: **Phase 2, switching perception to stereo.** The monocular pipeline (video replay → Metric3D V2 depth → ORB-SLAM3 localization → GPU elevation mapping) and the Gazebo sim pipeline both run end to end. Monocular depth-scale work is on hold; the active work item is the stereo pipeline (see [Current Target](#current-target)).

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

> **Monocular profiles (on hold).** The offline-video, depth-comparison and scale-probe profiles below belong to the monocular path. Keep them runnable. Stereo profiles get added here as the [stereo pipeline steps](#current-target) land.

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

**Stereo simulation** (ZED 2i rig on the textured benchmark: stereo Step 1):
```bash
docker compose --env-file stereo-sim.env \
  -f docker-compose-stereo-sim.yaml up -d --build
tmux attach -t quadruped_test_controller   # drive it
```

Publishes `/stereo/{left,right}/image_rect`, `/stereo/depth_gt/image_raw`, their `camera_info`, and `/stereo/imu/data`. Nothing consumes them until `stereo_depth` (Step 2).

**Depth scale probe** (Alternative 1 — live ground-plane scale estimate against the production Metric3D node):
```bash
docker compose --env-file depth-scale-probe.env \
  -f docker-compose-depth-scale-probe.yaml up -d --build
```

Runs `monocular_depth` (Metric3D V2, not the `depth_comparison` harness) on Gazebo's camera under `LOCALIZATION_MODE=ground_truth`, bridges its depth image to a point cloud via `depth_image_proc`, and feeds that to `ground_plane_estimation` with `GROUND_PLANE_HEIGHT_REFERENCE_FRAME=odom` — since `odom`'s z=0 plane is the true ground in this mode, `/ground_plane/scale` becomes a live per-frame estimate of Metric3D's depth scale error, no ground-truth depth image required. See `documentation/ground_plane_estimation.md` for the derivation.

**Odometry scale probe** (Alternative 2 — the same scale estimated from motion instead of ground geometry):
```bash
docker compose --env-file odometry-scale-probe.env \
  -f docker-compose-odometry-scale-probe.yaml up -d --build
tmux attach -t quadruped_test_controller   # the probe needs motion to publish
```

Same Gazebo + Metric3D + `depth_image_proc` front end as Alternative 1, then `lidar_odometry` runs KISS-ICP on the *monocular* cloud (`/depth/points`) so `/odom_lidar` inherits its scale error, and `odometry_scale_estimation` compares that against `/odom_lko` from `quadruped_leg_odometry` — the ratio of the two motions is the depth scale. `pointcloud_scale_correction` closes the loop into `/depth/points_corrected`. The profile runs two estimate channels: `/odom_scale/odom_lidar/scale` is the measurement, and `/odom_scale/odom_ground_truth/scale` compares Gazebo's true motion against the same reference, so it should sit at 1.0 and its deviation is the reference's own error — the noise floor below which the measurement means nothing. Metric3D's ~0.75 Hz on this host is what KISS-ICP has to register across, so drive slowly. See `documentation/odometry_scale_estimation.md`.

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
| `camera_link_optical` | Camera optical frame (z forward, x right, y down) |
| ZED driver frames | Stereo camera body, left/right optical frames, ZED IMU (planned; names set by the driver, see rules below) |
| `imu_link` | IMU sensor frame |
 
**Never invent frame names** (`robot`, `world`, `camera_frame`, `base`, `map_frame`). Use the names above. New sensor frames follow `<sensor>_link` (rigid body) and `<sensor>_link_optical` (optical frame), matching the existing `camera_link_optical`.

**Exception: driver-owned frames keep the driver's names.** The ZED wrapper stamps images, depth and IMU with frames derived from its `camera_name` (e.g. `zed_camera_link` plus left/right optical frames and an IMU frame). Use them verbatim, copied from the pinned wrapper's `zed_macro.urdf.xacro` rather than typed from memory, and make Gazebo emit the same names so sim and real share one frame set.
 
### Authoritative TF Tree
 
```
map
└── odom                     (publisher: robot_localization EKF; Phase 1: static TF)
    └── base_link            (publisher: robot_localization EKF / ros2_control)
        ├── base_footprint   (publisher: robot_state_publisher, static)
        ├── imu_link         (publisher: robot_state_publisher, static)
        ├── camera_link      (publisher: robot_state_publisher, static)
        │   └── camera_link_optical (publisher: robot_state_publisher, static)
        ├── zed_camera_link  (planned; robot_state_publisher, static, via the ZED xacro macro)
        │   └── [ZED left/right camera + optical frames, ZED IMU frame: driver-defined names]
        └── [leg frames]     (publisher: robot_state_publisher, from joint states)
```
 
Rules:
- **Each TF edge has exactly one publisher.** Adding a publisher means removing the old one. Duplicate publishers cause nondeterministic tf2 conflicts.
- ORB-SLAM3 publishes **`map→odom`**, not `map→base_link`. It corrects drift via the odom frame, leaving `odom→base_link` to odometry/EKF.
- Static transforms come from `robot_state_publisher` or `static_transform_publisher` — never a dynamic publisher emitting fixed values.
- **The ZED wrapper publishes no TF.** Launch it with `publish_urdf:=false` (the robot's `robot_state_publisher` owns the camera subtree through the ZED xacro macro), `pos_tracking.publish_tf:=false` and `pos_tracking.publish_map_tf:=false` (the EKF owns `odom→base_link`; SLAM / the global EKF owns `map→odom`). Its positional-tracking odometry may still be published as a topic for comparison.
- `elevation_mapping_cupy` fixed frame is **`map`** in offline-video mode (ORB-SLAM3 owns `map→base_link` directly). In Gazebo sim with dual-EKF, use **`odom`** — elevation mapping must run in a *smooth* frame; `map` frame discontinuities from loop closure would corrupt accumulated point clouds.
- **Point clouds from `depth_image_proc` are in `camera_link_optical` and carry the source image timestamp. Never restamp or reframe them.** This is the single most common Step 4 failure.
---
 
## Offline Video Workflow (monocular, on hold)

> For stereo, ZED SVO2 recordings played back through the ZED wrapper replace this bench. Playback gives the same topics as the live camera, IMU included (stereo Step 5). The rules below still apply whenever the monocular profile is run.
 
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
 
### Stereo topics (planned)

The ZED driver and the Gazebo stereo pair publish the same canonical names. The ZED wrapper's own topic names change between wrapper releases, so they are remapped onto these names at launch and never consumed directly.

| Role | Gazebo stereo pair | ZED 2i (live or SVO2) |
|---|---|---|
| Left / right rectified image | `/stereo/{left,right}/image_rect` (renders are already rectified) | ZED rectified images, remapped |
| Left / right camera info | `/stereo/{left,right}/camera_info` | ZED camera info, remapped |
| Depth (metres, registered to left) | `/depth/image_raw` from `stereo_depth` | ZED SDK depth, remapped |
| Depth confidence | `/depth/confidence` | ZED confidence map, remapped |
| Depth camera info | `/depth/camera_info` | ZED depth camera info, remapped |
| Point cloud | `/depth/points` (`depth_image_proc`) | `/depth/points` (`depth_image_proc`, not the ZED's own cloud, so the cloud path is identical) |
| Ground-truth depth (sim only) | `/stereo/depth_gt/image_raw` + `camera_info` (left-registered) | none |
| Camera IMU | `/stereo/imu/data` (200 Hz, frame `zed_imu_link`) | ZED IMU, remapped (`sensors_pub_rate: 200`) |
| Body IMU | `/imu/data` (`imu_link`) | Robot IMU. The camera IMU stays on its own topic: it is hardware-synced to the images, which ORB-SLAM3 stereo-inertial needs |

Only one node may publish `/depth/image_raw`: never run `monocular_depth` together with a stereo depth source.

All topic names must be remappable via launch arguments. Nodes that hardcode topic names break the sim/real switch.
 
> **Critical**: Gazebo depth is ground truth. Real depth is *inferred* and carries
> scale/shift error that varies per frame. **Gazebo cannot validate the perception
> stack.** Geometry, integration, and planning logic are testable in sim; the depth network,
> ORB-SLAM3, and real camera behaviour (auto-exposure, rolling shutter, motion
> blur) require recorded footage via the offline video pipeline as a separate test bench.
>
> **Stereo partly changes this.** Sim stereo runs a real matcher on rendered images, so Gazebo *can* validate the stereo geometry: baseline, rectification, disparity→depth, frames, and the matcher on textured surfaces. Its error is still only a lower bound (renders are noise-free, perfectly rectified and synchronized). Flat-shaded Gazebo surfaces have no texture to match, so benchmark worlds need textured materials. Real optics still need recorded footage (ZED SVO2).
 
---


## Architecture

### Full Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│  PERCEPTION                   (stereo; monocular path on hold)  │
│                                                                 │
│  ZED 2i (live / SVO2) ──→ ZED SDK depth ──→ /depth/image_raw    │
│  Gazebo GT + noise    ──→ stereo_depth  ──→ /depth/confidence   │
│  (surface normals: planned, derived from the depth image)       │
│                                                                 │
│  /depth/image_raw + /depth/camera_info                          │
│    └──→ depth_image_proc ──→ /depth/points (PointCloud2)        │
└─────────────────────────────────────────────────────────────────┘
                          │ /depth/points
                          │ /depth/surface_normals
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│  MAPPING                                                        │
│                                                                 │
│  elevation_mapping_cupy                                         │
│    ← /depth/points (covariance-weighted fusion, GPU)            │
│    ← /depth/surface_normals (slope/roughness inputs)            │
│    ← /odom (robot pose)                                         │
│    → /elevation_map  (grid_map)                                 │
│                                                                 │
│  traversability_estimation                                      │
│    ← /elevation_map                                             │
│    → /traversability_map  (slope + roughness + step height)     │
│    → /traversability_costmap  (nav_msgs/OccupancyGrid)          │
└─────────────────────────────────────────────────────────────────┘
                          │ /traversability_costmap
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│  LOCALIZATION                          (Phase 2+)               │
│                                                                 │
│  Leg Odometry (LKO) ──→ /odom_lko  (joint states + IMU)         │
│  ORB-SLAM3 (stereo) ──→ /odom_slam (loop-closure drift corr.)   │
│                                                                 │
│  robot_localization EKF                                         │
│    ← /odom_lko + /odom_slam + /imu/data                         │
│    → /odom  (fused, authoritative)                              │
│    → /tf: map → odom                                            │
│                                                                 │
│  Phase 1 fallback: static map→odom TF (fake localisation)       │
└─────────────────────────────────────────────────────────────────┘
                          │ /odom, /tf
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│  PLANNING & CONTROL                                             │
│                                                                 │
│  Nav2 (SmacPlannerLattice global, MPPI local)                   │
│    ← /traversability_costmap                                    │
│    ← /odom, /tf                                                 │
│    → /cmd_vel  (Twist: linear.x/y + angular.z)                  │
│    goal: /navigate_to_pose action (RViz2 or QGC mission)        │
└─────────────────────────────────────────────────────────────────┘
                          │ /cmd_vel
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│  LOCOMOTION                                                     │
│                                                                 │
│  Phase 1–2: Bézier-swing trot (analytic IK, 100 Hz)             │
│  Phase 3:   Perceptive RL policy (Isaac Lab → TensorRT/Jetson)  │
│               ← /cmd_vel + /elevation_map (proprioception)      │
│               → /quadruped_joint_controller/commands            │
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
| `monocular_depth` | `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` | 2+ | Metric3D V2 → metric depth (**on hold**: monocular path) |
| `zed_camera` (planned) | Jazzy + ZED SDK (needs CUDA; base fixed in stereo Step 3) | 2+ | ZED 2i driver (zed-ros2-wrapper): rectified stereo, SDK depth + confidence, IMU; live or SVO2 playback. Publishes no TF |
| `stereo_depth` (planned) | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 1+ | Sim depth source, since the ZED SDK can't ingest Gazebo images: `gt_noise` (GT depth + ZED-like error model, default) or `sgbm` (OpenCV matcher on the rendered pair) → `/depth/image_raw` + `/depth/confidence` |
| `depth_comparison` | `nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04` | 2+ | N depth models on identical frames → depth + cloud per model |
| `depth_image_proc` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 1+ | Depth → point cloud conversion |
| `elevation_mapping_cupy` | `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` | 1+ | GPU elevation mapping |
| `traversability_estimation` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 2+ | Slope/roughness/step costmap from elevation map |
| `localization` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 2+ | ORB-SLAM3 + LKO → robot_localization EKF |
| `lidar_odometry` | `osrf/ros:jazzy-desktop-full` | 2+ | KISS-ICP LiDAR odometry → `/odom_lidar` (metric reference / multi-vehicle odom source) |
| `nav2_mppi_navigation` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 1+ | Nav2 + MPPI, traversability costmap consumer |
| `quadruped_gait_controller` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 1–2 | Spot Micro Bézier-swing trot/crawl + whole-body IK; joint commands only |
| `quadruped_rl_controller` | `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` | 3 | TensorRT perceptive RL policy from Isaac Lab |
| `quadruped_test_controller` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | 1+ | Keyboard teleop |
| `video_camera_sim` | `ros:jazzy-ros-base` | 2+ | Plays pre-recorded video as camera topics for offline testing (monocular bench, on hold) |

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
Stereo Camera: ZED 2i via zed-ros2-wrapper (live, or SVO2 playback offline)   [planned]
  → /stereo/{left,right}/image_rect, /stereo/{left,right}/camera_info
  → /depth/image_raw      (ZED SDK stereo depth, registered to left, metres)
  → /depth/confidence, /depth/camera_info
  → /stereo/imu/data      (ZED onboard IMU, image-synced)
  (Gazebo: stereo camera pair + camera IMU → same image / IMU topics;
   GT depth → stereo_depth (ZED-like error model) → same depth topics)

[On hold: monocular path, still buildable; never run alongside stereo depth]
Video Camera Sim (offline testing, replaces live camera)
  ← pre-recorded video file
  → /camera/image_raw, /camera/camera_info
Monocular Depth (Metric3D V2)
  ← /camera/image_raw + /camera/camera_info  (intrinsics = metric scale)
  → /depth/image_raw      (metric, in metres)

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
  ← /stereo/{left,right}/image_rect + /imu/data  (ORB-SLAM3 IMU_STEREO, planned;
     first step: RGBD with left image + stereo depth)
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
  → /quadruped_joint_controller/commands only — /odom_lko is published
    by quadruped_leg_odometry (joint states + IMU), not by this node
```

---

## Key Source Files

> **Monocular path, on hold** (keep building; not part of the stereo pipeline): `monocular_depth`, `video_camera_sim`, `ground_plane_estimation`, `odometry_scale_estimation`, `pointcloud_scale_correction`, and `lidar_odometry` in its monodepth-cloud role. `depth_comparison` stays active: stereo backends are added there (stereo Step 2).

- `src/quadruped_kinematics/` — Spot Micro geometry (`SPOT_MICRO`: 0.186 × 0.078 m hip pivots, 55 mm hip link, 107.5 mm upper leg, 130 mm lower leg), closed-form leg IK/FK/Jacobian and whole-body IK (body pose over fixed footholds). Pure numpy, shared by the gait controller and leg odometry; its tests parse the URDF xacro and fail if the numbers diverge. Joint order contract: `JOINT_ORDER`.
- `src/quadruped_gait_controller/` — Spot Micro gait node (`gait_node.py` + pure-python `gait.py`). Consumes `/cmd_vel`, publishes joint commands only — leg kinematics odometry (`/odom_lko`) comes from `src/quadruped_leg_odometry/`, not this node. Bézier-swing gait (Hyun et al. 2014 curve) with per-leg stride vectors (`v + ω × r`, so yaw = tangential strides), `trot` (0.4 s cycle, duty 0.7, 20 mm lift; default) or `walk` (1.0 s crawl, duty 0.75, body shifted into the support triangle). A zero command finishes the cycle before standing. All tuning is ROS parameters (`config/gait_params.yaml`); per-gait defaults live in `GAIT_LIBRARY`. Validated headless in Gazebo: straight at 0.2–0.3 m/s with <2° roll/pitch, turns at the commanded rate. A duty-0.5 flight trot falls under position control — don't go back to it.
- `src/quadruped_gazebo/urdf/quadruped.urdf.xacro` — Spot Micro URDF: 12 joints (hip_roll, hip_pitch, knee per leg; all knees backward), 2.82 kg, 3.4 N·m / 7 rad/s servo limits, depth camera, IMU, RGB camera. The stand pose (`initial_value`) is derived from the geometry and `stand_height` (0.16 m) so the robot spawns standing; link lengths must match `quadruped_kinematics` (tested). `gazebo.launch.py headless:=true` runs the server only (CI / no display).
  `camera_rig:=mono|stereo` selects the legacy front/depth camera pair or the simulated ZED 2i (driver-named frames, `/stereo/*` topics, camera IMU; see Current Target Step 1). On the real robot use `camera_rig:=stereo publish_zed_imu_link:=false`.
- `src/quadruped_gazebo/config/ros2_control.yaml` — JointGroupPositionController mapping for all 12 joints.
- `src/quadruped_gazebo/launch/gazebo.launch.py` — Spawns robot, loads controllers, starts ROS-Gazebo bridges (both rigs' topics; the unused rig is silent).
- `src/quadruped_gazebo/quadruped_gazebo/benchmark_generator.py` — Deterministic benchmark worlds from `config/benchmarks/*.yaml`: terrain mesh, obstacles, and seeded procedural textures (`textures:` block; launch override `textures:=config|true|false`).
- `src/video_camera_sim/` — Plays pre-recorded video files as `/camera/image_raw` + `/camera/camera_info` for offline testing of perception and localization pipelines. Configurable FPS, loop, and camera intrinsics.
- `src/monocular_depth/` — (Phase 2) Metric3D V2 inference node — the production depth source. Intrinsics-conditioned, so it needs CameraInfo and drops frames until one arrives. TensorRT FP16 export for Jetson is still to come.
- `src/depth_comparison/` — Runs N depth backends on the *same* RGB frame and publishes one metric depth image + point cloud per model under `/depth_compare/<key>/`. Backends: Gazebo depth sensor (relay reference), Depth Anything V2 (VKITTI + Hypersim checkpoints), Metric3D V2. Add a model by implementing `DepthBackend` and registering it in `backends/__init__.py`. Shares the repo-root `compat/mmcv/` shim (a deliberate mmengine-backed stand-in for Metric3D's mmcv 1.x imports) with `monocular_depth`; both images therefore build from the repository root, not their own directory.
- `src/lidar_odometry/` — KISS-ICP LiDAR odometry from any `PointCloud2`; publishes `/odom_lidar` (`odom`→`base_link`, twist in `base_link`, drift-model covariance). Pure-lidar on purpose (sensor-agnostic, no IMU sync): the metric reference for the monocular scale work (Alternative 2) and an odometry source for lidar-equipped platforms of the vehicle contract. No TF by default (EKF owns `odom→base_link`). See `documentation/lidar_odometry.md` for the full algorithm survey (KISS-ICP vs. FAST-LIO2/Point-LIO/DLIO/LIO-SAM) and rationale.
- `src/ground_plane_estimation/` — RANSAC ground plane from any depth point cloud; publishes `/ground_plane[/<key>]/{distance,camera_height,scale,marker}`. The kinetic scale-hypothesis probe: with a height reference (`odom` z=0 in ground-truth sim, or a nominal standing height), `scale` = camera-to-plane distance / expected camera height ≈ per-frame monocular depth scale factor. One estimator per input topic; pairs with the depth comparison channels via `GROUND_PLANE_INPUT_TOPICS`.
- `src/pointcloud_scale_correction/` — Applies the ground-plane scale estimate back onto the point cloud: subscribes a `PointCloud2` + `/ground_plane/scale` (Float32), republishes `<input>_corrected` with the original header (frame + stamp untouched) plus `<output>/applied_scale`. Error model is pluggable (`correction_models.py`, pure numpy): `scalar` (`p/s`, default) or `affine` (`(z−b)/s` along each ray, camera-optical frame required) — switching is the `model` parameter (`odometry_scale_estimation`'s affine mode publishes a shift, but it is a displacement-space diagnostic, not the depth-space `b` — see that module's README before wiring it in). Estimates are EMA/median-smoothed and gated; `stale_policy` (hold/passthrough/drop) governs gaps. One node per input topic via `SCALE_CORRECTION_INPUT_TOPICS`, pairing with the depth comparison channels like `ground_plane_estimation`.
- `src/odometry_scale_estimation/` — The second kinetic probe (Alternative 2): compares a metric odometry stream (`/odom_lko`, leg kinematics + IMU) against a scale-bearing one (`/odom_lidar` — KISS-ICP fed the monodepth cloud via `LIDAR_POINTCLOUD_TOPIC=/depth/points`) over a sliding window (default 5 s) and publishes `/odom_scale[/<key>]/{scale,shift,residual,ref_motion,marker}`. Compares matched displacement chords (frame-invariant, so no extrinsic alignment; reference interpolated at estimate stamps), fit models pluggable in `trajectory_compare.py` (pure numpy): `scalar` (`chord_est = s·chord_ref`, default) or `affine` (`… + b`; needs chord-length spread, else rejected). Raw per-window output — smoothing/gating live in `pointcloud_scale_correction`, same contract as the ground-plane probe. Publishes nothing while standing (`min_motion` excitation gate); `ref_motion` distinguishes "standing" from "broken". See `documentation/odometry_scale_estimation.md`.
- `src/elevation_mapping_cupy_pkg/` — GPU elevation map (3m×3m, 5cm resolution, 2 Hz). Ingests point cloud + surface normals for covariance-weighted fusion. Vendored `elevation_mapping_cupy` repo.
- `src/traversability_estimation/` — (Phase 2) Computes traversability costmap from elevation map: slope angle, roughness (height variance), step height against leg reach limits. Publishes `nav_msgs/OccupancyGrid`.
- `src/localization/` — (Phase 2) ORB-SLAM3 (the node supports MONOCULAR and RGBD today; STEREO / IMU_STEREO are planned for the ZED 2i) + `robot_localization` EKF fusing LKO + SLAM + IMU. LKO is the primary short-horizon source; SLAM provides loop-closure drift correction.
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
| ORB-SLAM3 (stereo-inertial on the ZED 2i; RGBD with stereo depth as the first step) | Loop closure, long-horizon drift correction | Fails on texture-poor / dark environments |
| IMU (the ZED 2i's onboard IMU on real hardware) | Angular velocity + gravity vector | No position, biases over time |
| `robot_localization` EKF | Fuses all three → `/odom` | Single point of failure; tune noise covariances carefully |

LKO is the fallback when vision degrades. SLAM loop closure corrects accumulated LKO drift. Neither is used alone.

---

## Locomotion Strategy

| Phase | Approach | Tradeoffs |
|---|---|---|
| 1–2 | Bézier-swing trot / crawl, analytic whole-body IK (Spot Micro) | Simple, reliable on flat/mildly uneven terrain; open-loop, no terrain adaptation |
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
3. **Give the Dockerfile a non-root `USER` matching the host UID/GID** — see [Non-Root Container User](#non-root-container-user-required) below. Not optional: every module that bind-mounts the repo into `/work/ws` and runs `colcon build` there shares that mount with every other module's container, and a root-running container poisons the shared `log/`/`build`/`install/` dirs for everyone else.
4. Add `include: - path: src/<module>/docker-compose.yaml` to root `docker-compose.yaml`.
5. Add the container name to the `CONTAINERS` array in `tmux-attach.sh`.
6. Add at least one launch smoke test (node starts, required topics appear within 10s).

### Non-Root Container User (required)

Every module's Dockerfile bind-mounts the repo root to `/work/ws` and (usually) runs `colcon build` there against the same host directory every other module also builds into. If one container runs as root and another as a non-root user, whichever's `colcon build` claims a given `log/build_<timestamp>` directory first sets its ownership, and the other gets `PermissionError: [Errno 13]` writing its own log. **Every Dockerfile must end with a non-root `USER` whose UID/GID matches the host** (default 1000:1000), no exceptions — including modules that don't build anything against the shared mount, since the next thing added to that Dockerfile might.

Copy this block verbatim (adjust only if the module's base image already ships a `USER_UID`-matching user under a different flow — check with `id -u`/`id -g` before deviating):

```dockerfile
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# ... apt installs etc. must include `sudo` in the package list ...

# Non-root user matching host UID/GID. Guard on id -u: Ubuntu 24.04 base
# images already ship a UID-1000 "ubuntu" user, so useradd is a no-op there
# — do not assume it always runs.
RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME}; fi \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

WORKDIR /work/ws
USER ${USERNAME}
```

If the module needs `/dev/dri` (any GUI/GPU-rendering container, e.g. `quadruped_gazebo`), also join the host's `video`/`render` groups — **use `usermod -aG`, not `useradd -G`**, because the `useradd` step above is frequently skipped (base image already has the user) and `-G` on a skipped `useradd` silently does nothing:

```dockerfile
ARG VIDEO_GID=44
ARG RENDER_GID=109
# ...inside the same RUN block, after ensuring the groups/user exist above...
    && usermod -aG $(getent group ${VIDEO_GID} | cut -d: -f1),$(getent group ${RENDER_GID} | cut -d: -f1) ${USERNAME} \
```

Check the actual host GIDs with `getent group video render` before trusting the 44/109 defaults — they're Debian/Ubuntu conventional but not guaranteed.

Exceptions: a container that mounts the workspace `:ro` and never runs `colcon build` against it (e.g. `rviz`) doesn't participate in the race and doesn't strictly need this — but add it anyway unless there's a specific reason not to, so the whole repo stays consistent.

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

- **Gazebo rendering is not on the GPU in the stock containers.** The host's NVIDIA runtime runs in CDI mode and injects compute libraries only; `libEGL_nvidia` / `libGLX_nvidia` and the glvnd vendor files are missing in the container, and `quadruped_gazebo`'s compose forces `MESA_LOADER_DRIVER_OVERRIDE=llvmpipe`. Cameras therefore render on the CPU. Headless validation (2026-11-19) ran on the GPU by staging the host's `libEGL_nvidia.so.0`, `libnvidia-{eglcore,glsi,gpucomp,glcore,tls,glvkspirv}` under `build/` and setting `LD_LIBRARY_PATH` + `__EGL_VENDOR_LIBRARY_FILENAMES`. Even then the textured benchmark world runs at real-time factor ~0.55, for both rigs; the stereo rig itself costs almost nothing extra.

- **Stereo decided (ZED 2i, 2.1 mm)**: replaces monocular-only. NVIDIA Isaac ROS (`isaac_ros_dnn_stereo_depth` / ESS, FoundationStereo, cuVSLAM) remains the main alternative to the ZED SDK for depth and VIO on Jetson.
- **2.1 mm lens trade-off**: the wide FOV (~101°×69° at HD720) covers the near field: with the camera level at ~0.17 m, the ground is visible from ~0.25 m, and rated depth starts at 0.3 m. The cost is angular resolution. fx ≈ 528 px gives σ_Z ≈ 4 mm at 1 m, ~3.6 cm at 3 m and ~10 cm at 5 m (assuming σ_d = 0.25 px), so useful mapping range on the 5 cm grid is roughly 3.5 m.
- **Sim depth is modelled, not matched**: the ZED SDK accepts real cameras, SVO files and Isaac Sim streams only, not Gazebo. Sim depth is ground truth plus a ZED-like error model, so sim results for depth-sensitive code are only as good as that model. Its parameters must be fitted to SVO2 recordings (stereo Step 5), and the `sgbm` backend provides a pessimistic bound.
- **ZED SDK is closed source** and needs an NVIDIA GPU. Keep it behind the canonical `/stereo/*` and `/depth/*` topics so an open matcher (`stereo_depth`) and open VIO (ORB-SLAM3 / cuVSLAM) can replace it without touching downstream modules.
- **MPPI motion model**: Currently `Omni` for holonomic quadruped motion. Tune `vy_max` to match actual lateral gait speed.
- **Traversability costmap resolution**: 5cm grid at 3m×3m is appropriate for indoor/controlled outdoor. Expand to 10m×10m for open terrain but watch GPU memory on Orin Nano.
- **ORB-SLAM3 monocular scale** (on hold with the monocular path; stereo takes metric scale from the baseline): In `MONOCULAR` mode, ORB-SLAM3 scale is metric-consistent only if the monocular depth output is metric-accurate. In `RGBD` mode with monocular depth as the D channel, scale comes from the depth network. Validate carefully on real hardware.
- **Elevation map frame in offline mode**: Using `map` frame works when ORB-SLAM3 drift is small. For longer sequences with loop closures, the map frame may have discontinuities that affect elevation map coherence.
- **QGroundControl integration**: Planned. Requires MAVLink bridge (`MAVROS` or `px4_ros_com`) publishing mission waypoints as `nav2_msgs/action/NavigateThroughPoses`.




## Current Target

**Stereo pipeline on a ZED 2i (2.1 mm lens).** Monocular depth-scale work is on hold; its record is kept verbatim below, so resume from there.

### Stereo Pipeline

Decisions (2026-11-18):
- **Lens: 2.1 mm.** The 4 mm lens's 1.5 m minimum depth would leave most of the visible ground unmeasured on a robot this low.
- **Hardware depth: ZED SDK** (GPU, has a confidence map). Downstream sees only `/depth/*`, so an open matcher can replace it later without changes.
- **Sim depth: ground truth + a ZED-like error model.** The ZED SDK cannot ingest Gazebo images (its wrapper's `sim_mode` takes Isaac Sim streams only), so sim reproduces the ZED's *output contract*, not its algorithm. An OpenCV SGBM backend on the rendered pair is the pessimistic counterpart. Real ZED depth should fall between the two: GT + noise has no matching failures, while SGBM has holes and no learned fill.
- **SLAM: ORB-SLAM3 stereo-inertial** with the ZED's IMU. ZED positional tracking is a comparison channel only; cuVSLAM is the fallback if ORB-SLAM3 is too slow on Orin.

Camera facts the plan depends on (ZED 2i, 2.1 mm):
- 12 cm baseline; onboard IMU; USB 3; rolling shutter; rated depth 0.3–20 m.
- At HD720 (1280×720), the nominal fx is 528 px, so the FOV is ~101°×69°. The 110°×70° datasheet figure is for the full sensor. Per-unit factory calibration overrides the nominal value: trust the driver's `camera_info`.
- Depth noise is σ_Z ≈ Z²·σ_d/(f·B) ≈ 0.004·Z² m (σ_d = 0.25 px): ~4 mm at 1 m, ~3.6 cm at 3 m, ~10 cm at 5 m. It reaches the 5 cm map grid at ~3.5 m.
- With the camera level at ~0.17 m, the ground is visible from ~0.25 m. The whole visible ground is within rated range.

- [x] Step 0: Docs: stereo direction in CLAUDE.md, monocular path on hold, lens and depth-source decisions.
- [x] Step 1: Sim stereo rig. `camera_rig:=stereo` (URDF / `gazebo.launch.py` / `simulation_benchmark.launch.py`; default `mono` until Step 2 gives the default sim a depth source). Profile: `stereo-sim.env` + `docker-compose-stereo-sim.yaml`.
      - The ZED 2i on top of the head (229 g, 175×30×43 mm), with the driver's frames (`zed_camera_link → zed_camera_center → zed_{left,right}_camera_frame[_optical]`, `zed_left_camera_frame → zed_imu_link`), offsets from Stereolabs' `zed_macro.urdf.xacro`. Left camera 0.205 m above ground.
      - Topics: `/stereo/{left,right}/image_rect` + `camera_info` and `/stereo/depth_gt/image_raw` + `camera_info` (ground truth, left-registered), all at 15 Hz with identical stamps. `/stereo/imu/data` at 200 Hz.
      - Intrinsics: fx = fy = 528, cx = 640, cy = 360; the right camera's P carries Tx = −fx·B = −63.36. `test_stereo_rig.py` pins the frame names, baseline and intrinsics.
      - IMU noise is a **placeholder** (community ZED 2 Kalibr set) to replace with an Allan-variance fit. On the real camera set `sensors_pub_rate: 200` to match, and launch the URDF with `publish_zed_imu_link:=false` (the driver publishes that edge from factory calibration).
      - The benchmark generator writes seeded, tileable procedural textures (terrain + every obstacle). `textures:=config|true|false` overrides the scenario. The monocular profiles pin `camera_rig:=mono textures:=false` so their results stay reproducible.
      - The camera moves the CoM ~1.1 cm forward. Walking forward is unaffected, but turning in place lost ~10% of the commanded yaw and roll/pitch roughly doubled; `GAIT_BODY_X=-0.011` (gait `body_x`, set in `stereo-sim.env`) restores the camera-less baseline (headless check, 2026-11-19).
- [ ] Step 2: `stereo_depth` node (sim depth source) → `/depth/image_raw` + `/depth/confidence` + `/depth/camera_info`.
      - Backend `gt_noise` (default): GT depth → clamp to the ZED range (0.3–20 m, configurable) → disparity-space noise with σ_Z(Z) above, spatially correlated through a configurable low-pass filter (ZED neural depth is smooth, not per-pixel white) → invalid pixels at depth discontinuities and on configurable dropout → confidence derived from σ_Z.
      - Backend `sgbm`: OpenCV SGBM on the rendered pair; also registered in `depth_comparison` so both are scored against GT per range bin.
      - The existing sim pipeline (depth_image_proc → elevation mapping) then runs on stereo depth.
- [ ] Step 3: ZED driver module `zed_camera`: Jazzy + ZED SDK + zed-ros2-wrapper, non-root user, USB passthrough. Pin the SDK and wrapper versions and confirm that pair supports Jazzy and JetPack 6.
      Launch with `general.camera_name: zed`, `publish_urdf:=false`, `pos_tracking.publish_tf:=false`, `pos_tracking.publish_map_tf:=false`, `sensors.sensors_pub_rate: 200`, and remap onto the canonical topics. Copy the factory-calibrated intrinsics and IMU extrinsics into the sim rig (Step 1) once the camera is in hand.
- [ ] Step 4: Localization, developed in sim first (the rig from Step 1 provides the stereo pair + IMU), then on SVO2.
      First, ORB-SLAM3 RGBD with the left image + `/depth/image_raw` (launch remaps only, no node change). Then add `STEREO` / `IMU_STEREO` to `orbslam3_rgbd_node` (IMU noise densities and camera-IMU extrinsics in the settings YAML).
      On hardware, log ZED positional tracking alongside as a comparison channel.
- [ ] Step 5: Offline stereo bench. Record SVO2 (stereo + IMU) at robot height and play it back through the wrapper into the full pipeline; this replaces iPhone videos as the real-camera test bench.
      Fit the `gt_noise` parameters (σ_d, correlation length, dropout) to real ZED depth, e.g. plane residuals on a flat floor, and feed them back into the sim.
- [ ] Step 6: Mapping on stereo. Elevation mapping on `/depth/points` with confidence-derived variance; check map extent against the ~3.5 m useful range.

Free sanity check from the on-hold work: run `ground_plane_estimation` on stereo depth. Its `scale` should sit at ~1.0, and a deviation means a calibration or frame error, not a model error.

### Monocular depth scale (on hold)

Original notes, kept as they were when the work was paused:

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

 Alternative 1 launch - docker compose --env-file depth-scale-probe.env   -f docker-compose-depth-scale-probe.yaml up -d
 Alternative 2 launch - docker compose --env-file odometry-scale-probe.env   -f docker-compose-odometry-scale-probe.yaml up -d 
