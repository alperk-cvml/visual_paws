# Visual Paws — Project Summary for Review

## Project Overview

**Visual Paws** is a ROS 2 autonomous navigation stack for a 12-DOF quadruped robot targeting real outdoor deployment. The key constraint is **camera-only perception** — no LiDAR. All depth estimation and terrain understanding comes from RGB imagery processed by neural networks running on an NVIDIA Jetson Orin Nano/NX edge computer.

**Primary goal:** A field-deployable quadruped that navigates complex outdoor terrain using only an RGB camera, running fully on edge hardware without any cloud dependency.

**Current state:** Phase 1 complete (Gazebo Harmonic simulation, fake localization, diagonal trot gait). Phase 2 (real hardware perception and localization) is in progress.

---

## Hardware Target

| Component | Spec |
|---|---|
| Compute | NVIDIA Jetson Orin Nano / NX |
| Architecture | ARM64, JetPack 6.x, L4T r36.x |
| GPU | NVIDIA CUDA (TensorRT inference) |
| Robot | 12-DOF quadruped (3 joints/leg × 4 legs) |
| Sensors | RGB camera (single), IMU |
| No LiDAR | Depth inferred from monocular RGB via neural networks |

The choice of Jetson Orin is deliberate — NVIDIA provides a documented sim-to-real transfer path from Isaac Lab to Jetson for the Phase 3 RL locomotion policy, and the pre-built PyTorch + CUDA images (`dustynv/ros:jazzy-pytorch-l4t-r36.4.0`) avoid painful manual ARM64 build chains.

---

## System Architecture

The stack is divided into five vertical layers, each in its own Docker container. Data flows top-to-bottom:

```
RGB Camera
    │
    ▼
PERCEPTION (monocular_depth container)
  Depth Anything V2 (metric depth, metres)
    → /depth/image_raw  (32FC1)
    → /depth/surface_normals
    │
    ▼
POINT CLOUD (depth_image_proc container)
  depth_image_proc
    ← /depth/image_raw + /camera/camera_info
    → /depth/points  (PointCloud2)
    │
    ▼
MAPPING (elevation_mapping_cupy container)
  GPU elevation mapping (3m×3m, 5cm resolution, 2 Hz)
    → /elevation_map  (grid_map)
  Traversability estimation (slope + roughness + step height)
    → /traversability_costmap  (nav_msgs/OccupancyGrid)
    │
    ▼
LOCALIZATION (localization container)
  ORB-SLAM3 RGBD  → /odom_slam  (loop-closure-corrected)
  Gait controller → /odom_lko   (leg kinematics odometry)
  IMU             → /imu/data
  robot_localization EKF (50 Hz fusion)
    → /odom  (authoritative)
    → /tf: map → odom
    │
    ▼
PLANNING & CONTROL (nav2_mppi_navigation container)
  Nav2 + MPPI local planner
    ← /traversability_costmap, /odom, /tf
    → /cmd_vel  (Twist)
    │
    ▼
LOCOMOTION (quadruped_gait_controller container)
  Phase 1–2: Diagonal trot, analytic IK (50 Hz)
  Phase 3:   Isaac Lab RL policy → TensorRT/Jetson
    ← /cmd_vel
    → /quadruped_joint_controller/commands  (12 joint positions)
    → /odom_lko  (published by gait controller)
```

---

## Development Phases

| Phase | Status | Locomotion | Localization | Perception |
|---|---|---|---|---|
| **Phase 1** | Complete | Fixed diagonal trot (analytic IK) | Fake static TF | Gazebo depth camera |
| **Phase 2** | In progress | Fixed trot + terrain cost | ORB-SLAM3 + EKF | Depth Anything V2 (monocular) |
| **Phase 3** | Planned | Perceptive RL policy (Isaac Lab → TensorRT) | Same as Phase 2 | Same as Phase 2 |

---

## Container Layout

Every module is its own Docker container. The root `docker-compose.yaml` uses Compose `include:` to compose them. All containers share the host network (`network_mode: host`) with `ipc: host` for zero-configuration CycloneDDS discovery — no manual ROS_DOMAIN_ID routing or DNS needed.

| Container | Base Image | Role |
|---|---|---|
| `quadruped_gazebo` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | Gazebo Harmonic physics sim, robot spawn, sensor bridge |
| `depth_image_proc` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | Converts metric depth + camera_info → PointCloud2 |
| `elevation_mapping_cupy` | `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` | GPU elevation map (CuPy kernels) |
| `monocular_depth` | `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` | Depth Anything V2 inference (metric, outdoor weights) |
| `localization` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | ORB-SLAM3 RGBD + robot_localization EKF |
| `nav2_mppi_navigation` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | Nav2 + MPPI planner |
| `quadruped_gait_controller` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | Diagonal trot gait + analytic IK + LKO odometry |
| `quadruped_test_controller` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | Keyboard teleop (W/A/S/D + Q/E) |
| `video_camera_sim` | `ros:jazzy-ros-base` | Plays pre-recorded video as camera topics for offline testing |

Two base image tiers are used deliberately: CPU-only nodes get the lighter `jazzy-ros-base` image; GPU/inference nodes get `jazzy-pytorch` which ships pre-built CUDA + PyTorch for Jetson, avoiding ARM64 build complexity.

---

## Key Design Decisions

### Camera-Only Perception
No LiDAR. Metric depth is inferred via **Depth Anything V2** (ViT-S/B/L encoder, trained on Virtual KITTI for outdoor metric accuracy). Surface normals are estimated alongside depth. This allows slope and roughness classification without any active ranging sensor.

**Acknowledged risk:** Monocular metric depth is noisier than stereo or LiDAR. The RL policy in Phase 3 is chosen partly because it is more robust to noisy depth than classical MPC-based foothold planning (OCS2/legged_perceptive), which requires precise terrain geometry.

**Stereo fallback decision:** NVIDIA Isaac ROS stereo stack (cuVSLAM, FoundationStereo) would be significantly richer if a second camera is added. This is documented as an open decision — if cost allows, stereo should be reconsidered before Phase 2 hardware commitment.

### Modular, Not End-to-End
Each layer is independently replaceable. AI is injected only at the locomotion boundary (Phase 3 RL policy) and at the perception boundary (depth inference). Everything in between — elevation mapping, traversability, MPPI, EKF localization — is classical and fully debuggable without neural network involvement.

### Vehicle-Agnostic Above the Odometry Boundary
All platforms publish `nav_msgs/Odometry` + a traversability costmap. Elevation mapping, MPPI, and localization EKF are shared across vehicle types. Only the odometry source and drive controller differ per platform:

| Platform | Odometry | Drive Controller |
|---|---|---|
| Quadruped | LKO + ORB-SLAM3 EKF | Gait controller |
| Ackermann | Wheel encoder + GPS | ackermann_steering_controller |
| Tracked | Wheel encoder + IMU | Differential drive |
| USV | GPS + DVL | Thruster controller |

---

## Locomotion — Gait Controller

**Implementation:** `src/quadruped_gait_controller/quadruped_gait_controller/gait_node.py`

**Robot geometry:**
- Body: 250mm × 120mm × 55mm, 1.2 kg
- Thigh: 85mm, Calf: 95mm
- Hip abduction link: 40mm
- Joint limits: hip_roll ±0.6 rad, hip_pitch ±1.2 rad, knee −2.5 to −0.3 rad

**Gait:** Diagonal trot at 50 Hz. Phase A [0, 0.5): FL+RR swing; Phase B [0.5, 1): FR+RL swing. Stride period 0.4s, step height 15mm, max stride 80mm, max lateral 40mm.

**IK:** Analytic 3-DOF per leg. Hip roll uses coronal-plane geometry with the hip abduction offset. Hip pitch and knee use 2-link sagittal-plane IK (law of cosines). No numerical solver — closed-form, runs at 50 Hz with no iteration cost.

**Velocity commands:** Accepts `geometry_msgs/Twist` on `/cmd_vel`. Applies acceleration limits (0.8 m/s² linear, 2.0 rad/s² angular). Supports forward, lateral, and rotational velocity simultaneously (holonomic motion).

**Phase 3 upgrade path:** Replace with Isaac Lab RL policy exported to TensorRT. Policy receives `/cmd_vel` + `/elevation_map` (proprioception layer). NVIDIA provides a documented Jetson Orin transfer path for this.

---

## Perception — Monocular Depth

**Implementation:** `src/monocular_depth/monocular_depth/depth_node.py`

**Model:** Depth Anything V2 (metric variant, Virtual KITTI outdoor weights). Supports ViT-S (vits), ViT-B (vitb), ViT-L (vitl) via parameter. Default: ViT-S for Jetson power budget.

**I/O:**
- Subscribes: `/front_camera/image_raw` (RGB, `sensor_msgs/Image`)
- Publishes: `/depth/image_raw` (metric depth, `32FC1`, metres)
- Re-publishes: `/depth/camera_info` (timestamp-synced with depth)

**Rate limiting:** Configurable target FPS (default 10 Hz). Frames arriving faster than target are dropped before GPU dispatch. Logs average inference latency every 50 frames.

**Device fallback:** If CUDA requested but unavailable, falls back to CPU with a warning.

---

## Localization

**Implementation:** `src/localization/src/orbslam3_rgbd_node.cpp` + `src/localization/config/ekf.yaml`

### ORB-SLAM3 RGBD Node (C++)

Synchronizes RGB and depth images (ApproximateTime policy), runs ORB-SLAM3 tracking, and publishes `nav_msgs/Odometry` on `/odom_slam`.

**Camera configuration (sim):** 640×480, fx=fy=381.36, cx=320, cy=240, pinhole, no distortion. ORB: 1000 features, scale factor 1.2, 8 pyramid levels.

**Frame conversion:** ORB-SLAM3 outputs camera optical frame (X-right, Y-down, Z-forward). Node applies Rz(−π/2)·Rx(−π/2) to convert to ROS base_link convention (X-forward, Y-left, Z-up).

**Covariance:** Position variance 0.05 m², orientation variance 0.1 rad². Conservative — SLAM is accurate but can produce jump discontinuities on loop closure.

**Tracking state monitoring:** Suppresses odometry publication when tracking is LOST. Logs tracking rate, map point count, and keypoint count every 10 seconds.

### EKF Fusion (robot_localization)

Fuses three sources at 50 Hz:

| Source | Topic | Role | Config |
|---|---|---|---|
| Leg Kinematics Odometry | `/odom_lko` | Primary short-horizon, high-frequency | Differential integration (incremental) |
| ORB-SLAM3 | `/odom_slam` | Loop-closure drift correction, absolute pose | Direct (non-differential) |
| IMU | `/imu/data` | Angular velocity + linear acceleration | Gravity removal enabled |

**Process noise tuning:** Position 0.05–0.06, orientation 0.03–0.04, velocity 0.025, angular rate 0.01–0.02, acceleration 0.01–0.015. Tuned for outdoor quadruped locomotion with monocular depth noise.

**Phase 1 fallback:** Static `map→odom` TF published by the Gazebo launch file. The EKF container is Phase 2+.

---

## Planning — Nav2 + MPPI

**Implementation:** `src/nav2_mppi_navigation/nav2_mppi_navigation/config/nav2_mppi_params.yaml`

### MPPI Controller Configuration

| Parameter | Value | Note |
|---|---|---|
| `batch_size` | 2000 | Rollout samples per step |
| `time_steps` | 56 | Horizon steps |
| `model_dt` | 0.05 s | 20 Hz effective horizon |
| `motion_model` | DiffDrive → Omni | Must change to Omni for lateral quadruped motion |
| `vx_max` / `vx_min` | 0.5 / −0.35 m/s | Forward/backward limits |
| `vy_max` | 0.0 → 0.5 m/s | 0 now; set 0.5 when switching to Omni |
| `wz_max` | 1.9 rad/s | Rotational limit |
| `temperature` | 0.3 | MPPI smoothing |
| Controller frequency | 20 Hz | |

**Critics enabled:** ConstraintCritic, CostCritic (footprint-aware, collision_cost=1e6), GoalCritic, GoalAngleCritic, PathAlignCritic (weight 14.0), PathFollowCritic, PathAngleCritic, PreferForwardCritic.

**Known issue:** `motion_model` is currently `DiffDrive` — must be changed to `Omni` to enable lateral velocity commands that the quadruped gait controller can execute. This is a documented open item.

### Costmaps

- **Local:** 4m×4m rolling window, 5cm resolution, reads `/occupancy_grid_layer` from elevation mapping
- **Global:** 15m×15m rolling window, 5cm resolution, same source
- **Footprint:** 0.40m × 0.30m rectangular polygon, inflation radius 0.55m

### Global Planner

NavFn (Dijkstra) — sufficient for Phase 1/2. SmacPlannerLattice is the intended Phase 2 upgrade for terrain-aware global paths.

---

## Mapping — Elevation Mapping CuPy

GPU-accelerated elevation map using CuPy CUDA kernels. Ingests `PointCloud2` from depth_image_proc. Publishes a `grid_map_msgs/GridMap` (`/elevation_map`) and an `OccupancyGrid` (`/occupancy_grid`).

**Parameters (Phase 1):** 3m×3m, 5cm resolution, 2 Hz update. Phase 2 expansion planned to 10m×10m for open outdoor terrain (GPU memory constraint on Orin Nano).

**Traversability:** Slope angle, height variance (roughness), step height vs. leg reach limits → `nav_msgs/OccupancyGrid` on `/traversability_costmap`.

---

## Offline Testing — Video Camera Simulator

**Implementation:** `src/video_camera_sim/video_camera_sim/video_node.py`

Plays pre-recorded video files as a live camera feed, enabling offline development and testing of the monocular depth and ORB-SLAM3 pipelines without Gazebo or a physical robot.

**Features:**
- Reads MP4/AVI/MKV via OpenCV
- Publishes `/camera/image_raw` (BGR8) + `/camera/camera_info`
- Configurable FPS, resolution, looping
- Correct CameraInfo (intrinsic matrix) derived from URDF camera parameters
- Dynamic parameter updates at runtime (FPS, resolution, loop)
- Timestamps synchronized between image and camera_info messages

This is the primary development loop for Phase 2 perception work: record outdoor footage → play back → validate depth inference and SLAM without any physical hardware.

---

## Robot Model (URDF)

**File:** `src/quadruped_gazebo/urdf/quadruped.urdf.xacro`

**Body:** 250mm × 120mm × 55mm, 1.2 kg

**Legs (×4, mirrored):**
- Hip abduction link: 40mm, 0.08 kg
- Thigh: 85mm, 0.10 kg
- Calf: 95mm, 0.06 kg
- Foot sphere: 15mm radius, 0.02 kg

**Joints:** 12 total (hip_roll, hip_pitch, knee per leg). All controlled via `position_controllers/JointGroupPositionController` through `ros2_control`.

**Sensors (sim):**
- Front RGB camera: 640×480, hfov=80°, fx=fy=381.36
- Depth camera: Same intrinsics, publishes raw depth + PointCloud2
- IMU: Standard Gazebo plugin

**Gazebo bridge topics:** IMU, front camera (RGB + CameraInfo), depth camera (depth image + CameraInfo + PointCloud2), clock, TF from Gazebo model pose.

---

## Simulation Infrastructure

**Simulator:** Gazebo Harmonic (gz-sim), interfaced via `ros_gz_bridge` and `ros_gz_sim`.

**Launch sequence** (`gazebo.launch.py`):
1. Robot State Publisher (URDF → /robot_description)
2. Gazebo Harmonic with SDF world
3. Spawn robot via `ros_gz_sim create`
4. After spawn: load `joint_state_broadcaster` → then `quadruped_joint_controller`
5. Start ROS-Gazebo topic bridge
6. Publish static TFs: `quadruped/base_link→base_link`, `quadruped_world→odom`, `odom→map` (Phase 1 fake localization)
7. Optional RViz2

**tmux session management:** `tmux-attach.sh` creates one tmux session per container, each pre-configured with the correct `docker exec` command for that container's workspace.

---

## Key Software Stack

| Layer | Technology |
|---|---|
| OS | Ubuntu 24.04 (Jetson) / Ubuntu 24.04 (dev host) |
| Robot middleware | ROS 2 Jazzy |
| Simulation | Gazebo Harmonic |
| DDS | CycloneDDS (`rmw_cyclonedds_cpp`) |
| Languages | Python 3, C++ (ROS 2 nodes) |
| GPU inference | PyTorch (training), TensorRT FP16 (deployment) |
| Depth model | Depth Anything V2 (metric, ViT-S/B/L, Virtual KITTI weights) |
| SLAM | ORB-SLAM3 (RGBD mode) |
| Sensor fusion | robot_localization EKF |
| Navigation | Nav2 (MPPI local planner, NavFn global planner) |
| Elevation mapping | elevation_mapping_cupy (CuPy CUDA kernels) |
| Container runtime | Docker + Docker Compose v2 (`include:` federation) |

---

## Known Limitations & Open Decisions

| Item | Status | Detail |
|---|---|---|
| MPPI motion model | Bug / open item | Currently `DiffDrive`; must be `Omni` for lateral quadruped motion. `vy_max` set to 0 as guard. |
| ORB-SLAM3 depth input | Design tradeoff | Uses monocular depth as D channel — not a true stereo pair. Metric accuracy depends on Depth Anything V2 calibration. Requires real-hardware validation. |
| Stereo camera option | Open decision | Isaac ROS stereo stack (cuVSLAM, FoundationStereo) would be richer. Reconsider if second camera cost is acceptable before Phase 2 hardware commit. |
| Traversability costmap resolution | Scaling concern | 3m×3m at 5cm is right for controlled outdoor. Expanding to 10m×10m for open terrain needs GPU memory audit on Orin Nano. |
| QGroundControl integration | Planned | Requires MAVLink bridge (MAVROS or px4_ros_com) publishing mission waypoints as `nav2_msgs/action/NavigateThroughPoses`. |
| Phase 3 RL policy training | Future | Requires Isaac Lab + Isaac Gym environment setup. Target: zero-shot transfer to Jetson Orin via TensorRT FP16 export. |

---

## Environment Variables

| Variable | Default | Purpose |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | Isolate ROS 2 DDS domain |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS middleware selection |
| `DISPLAY` / `XAUTHORITY` | from host | Gazebo GUI via X11 forwarding |
| `NVIDIA_VISIBLE_DEVICES` | `all` | GPU access in CUDA containers |
| `CUDA_VISIBLE_DEVICES` | `0` | Single GPU, avoids allocation conflicts |

---

## Adding a New Module

1. Create `src/<module>/` with `package.xml`, `docker-compose.yaml`, `CMakeLists.txt` or `setup.py`
2. Choose base image: `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` (CPU) or `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` (GPU/inference)
3. Add `include: - path: src/<module>/docker-compose.yaml` to root `docker-compose.yaml`
4. Add container name to `CONTAINERS` array in `tmux-attach.sh`
5. Add a launch smoke test: node starts, required topics appear within 10 seconds
