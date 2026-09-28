# Visual Paws — Project Summary for Review

## Project Overview

**Visual Paws** is a ROS 2 autonomous navigation stack for a 12-DOF quadruped robot targeting real outdoor deployment. The key constraint is **camera-only perception** — no LiDAR on the robot. All depth estimation and terrain understanding comes from RGB imagery processed by neural networks running on an NVIDIA Jetson Orin Nano/NX edge computer.

**Primary goal:** A field-deployable quadruped that navigates complex outdoor terrain using only an RGB camera, running fully on edge hardware without any cloud dependency.

**Current state:** Phase 1 complete (Gazebo Harmonic simulation, Spot Micro Bézier-swing gait, ground-truth localization). Phase 2 is in progress: the offline video pipeline runs end to end (video → Metric3D V2 depth → ORB-SLAM3 → GPU elevation map), and Metric3D V2 replaced Depth Anything V2 as the production depth source after a side-by-side model comparison in Gazebo. Monocular depth **scale error** is the open problem. The current detour tries to estimate the scale *kinetically*, while the robot runs, in two ways:

- **Alternative 1 — ground plane:** measured camera height above a RANSAC ground plane ÷ expected camera height.
- **Alternative 2 — motion:** KISS-ICP odometry on the monocular point cloud ÷ metric leg-kinematics + IMU odometry.

Both probes, and a correction node that applies their estimate back onto the point cloud, are implemented and runnable in Gazebo.

---

## Hardware Target

| Component | Spec |
|---|---|
| Compute | NVIDIA Jetson Orin Nano / NX |
| Architecture | ARM64, JetPack 6.x, L4T r36.x |
| GPU | NVIDIA CUDA (TensorRT inference) |
| Robot | Spot Micro-class 12-DOF quadruped (3 joints/leg × 4 legs), 2.82 kg |
| Sensors | RGB camera (single), IMU |
| No LiDAR | Depth inferred from monocular RGB via neural networks |

The choice of Jetson Orin is deliberate — NVIDIA provides a documented sim-to-real transfer path from Isaac Lab to Jetson for the Phase 3 RL locomotion policy, and the pre-built PyTorch + CUDA images (`dustynv/ros:jazzy-pytorch-l4t-r36.4.0`) avoid painful manual ARM64 build chains.

---

## System Architecture

The stack is divided into vertical layers, each in its own Docker container(s). Data flows top-to-bottom:

```
RGB Camera  (or video_camera_sim for offline footage)
    │
    ▼
PERCEPTION (monocular_depth container)
  Metric3D V2 (metric depth, metres, intrinsics-conditioned)
    → /depth/image_raw  (32FC1)
    (/depth/surface_normals planned — normal head not wired yet)
    │
    ▼
POINT CLOUD (depth_image_proc container)
  ← /depth/image_raw + /camera/camera_info
  → /depth/points  (PointCloud2)
    │
    ├──► SCALE PROBES (detour, see "Depth Scale Problem")
    │      ground_plane_estimation / lidar_odometry + odometry_scale_estimation
    │      → scale estimate → pointcloud_scale_correction → /depth/points_corrected
    ▼
MAPPING (elevation_mapping_cupy container)
  GPU elevation mapping (3m×3m, 5cm resolution, 2 Hz)
    → /elevation_map  (grid_map)
  Traversability estimation (planned: slope + roughness + step height)
    → /traversability_costmap  (nav_msgs/OccupancyGrid)
    │
    ▼
LOCALIZATION (localization + quadruped_leg_odometry containers)
  Leg odometry    → /odom_lko   (joint states + IMU)
  ORB-SLAM3 RGBD  → /odom_slam  (loop-closure-corrected)
  Local EKF  (50 Hz) → /odom,              TF odom → base_link
  Global EKF (30 Hz) → /odometry/global,   TF map → odom
    │
    ▼
PLANNING & CONTROL (nav2_mppi_navigation container)
  Nav2 + MPPI local planner
    ← costmap, /odom, /tf
    → /cmd_vel  (Twist)
    │
    ▼
LOCOMOTION (quadruped_gait_controller container)
  Phase 1–2: Bézier-swing trot/crawl, whole-body IK (100 Hz)
  Phase 3:   Isaac Lab RL policy → TensorRT/Jetson
    ← /cmd_vel
    → /quadruped_joint_controller/commands  (12 joint positions)
```

---

## Development Phases

| Phase | Status | Locomotion | Localization | Perception |
|---|---|---|---|---|
| **Phase 1** | Complete | Bézier-swing trot (Spot Micro, analytic IK) | Ground-truth TF from Gazebo | Gazebo depth camera |
| **Phase 2** | In progress | Fixed trot + terrain cost | LKO + ORB-SLAM3 + dual EKF | Metric3D V2 (monocular) + kinetic scale correction |
| **Phase 3** | Planned | Perceptive RL policy (Isaac Lab → TensorRT) | Same as Phase 2 | Same as Phase 2 |

---

## Container Layout

Every module is its own Docker container. The root `docker-compose.yaml` uses Compose `include:` to compose them. All containers share the host network (`network_mode: host`) with `ipc: host` for zero-configuration CycloneDDS discovery — no manual DNS or domain routing needed.

| Container | Base Image | Role |
|---|---|---|
| `quadruped_gazebo` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | Gazebo Harmonic physics sim, robot spawn, sensor bridge, ground-truth odometry |
| `rviz` | `osrf/ros:jazzy-desktop-full` | Standalone visualization for sim and offline pipelines |
| `monocular_depth` | `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` | Metric3D V2 inference (production depth source) |
| `depth_comparison` | `nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04` | N depth models on identical frames → depth + cloud per model |
| `depth_image_proc` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | Metric depth + camera_info → PointCloud2 |
| `ground_plane_estimation` | `osrf/ros:jazzy-desktop-full` | RANSAC ground plane → per-frame depth scale (Alternative 1) |
| `lidar_odometry` | `osrf/ros:jazzy-desktop-full` | KISS-ICP odometry from any PointCloud2 → `/odom_lidar` |
| `odometry_scale_estimation` | `osrf/ros:jazzy-desktop-full` | Compares two odometry streams → depth scale (Alternative 2) |
| `pointcloud_scale_correction` | `osrf/ros:jazzy-desktop-full` | Applies a scale estimate to the point cloud → `<input>_corrected` |
| `elevation_mapping_cupy` | `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` | GPU elevation map (CuPy kernels) |
| `localization` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | ORB-SLAM3 RGBD + robot_localization dual EKF |
| `quadruped_leg_odometry` | `osrf/ros:jazzy-desktop-full` | Leg kinematics + IMU odometry → `/odom_lko` |
| `nav2_mppi_navigation` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | Nav2 + MPPI planner |
| `quadruped_gait_controller` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | Spot Micro Bézier-swing trot/crawl + whole-body IK |
| `quadruped_test_controller` | `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` | Keyboard teleop (W/A/S/D + Q/E) |
| `video_camera_sim` | `ros:jazzy-ros-base` | Plays pre-recorded video as camera topics for offline testing |

Two base image tiers are used deliberately: CPU-only nodes get the lighter `jazzy-ros-base` image; GPU/inference nodes get `jazzy-pytorch`, which ships pre-built CUDA + PyTorch for Jetson and avoids ARM64 build complexity. The measurement modules added for the scale detour (`ground_plane_estimation`, `lidar_odometry`, `odometry_scale_estimation`, `pointcloud_scale_correction`, `quadruped_leg_odometry`) currently build on `osrf/ros:jazzy-desktop-full`, and `depth_comparison` on a stock CUDA image — dev-host images, not yet ported to the Jetson base images.

### Launch Profiles

Each profile is an env file paired with a compose entry point that selects a subset of the module composes:

| Profile | Command | Purpose |
|---|---|---|
| Gazebo sim | `docker compose up -d --build` | Full sim pipeline |
| Offline video | `docker compose --env-file offline-video.env up -d --build` | Recorded video → Metric3D → ORB-SLAM3 → elevation map |
| Depth comparison | `--env-file depth-comparison.env -f docker-compose-depth-comparison.yaml` | Gazebo GT vs. DA-V2 (VKITTI, Hypersim) vs. Metric3D-S on identical frames |
| Depth scale probe | `--env-file depth-scale-probe.env -f docker-compose-depth-scale-probe.yaml` | Alternative 1: live ground-plane scale of production Metric3D |
| Odometry scale probe | `--env-file odometry-scale-probe.env -f docker-compose-odometry-scale-probe.yaml` | Alternative 2: motion-based scale of production Metric3D |

The three Gazebo profiles run under `LOCALIZATION_MODE=ground_truth` on the deterministic `v0_geometry` benchmark world (seed 42).

---

## Key Design Decisions

### Camera-Only Perception
No LiDAR on the robot. Metric depth is inferred via **Metric3D V2** (ViT-S/L/g2 encoder), which predicts in a canonical camera space and recovers metric scale from the camera's focal length. This allows slope and roughness classification without any active ranging sensor. Metric3D also has a surface-normal head; it is not wired to `/depth/surface_normals` yet.

The `lidar_odometry` module does not change this: KISS-ICP there runs on the *monocular* point cloud as a measurement instrument, or on real LiDAR for other vehicle types.

**Acknowledged risk:** Monocular metric depth is noisier than stereo or LiDAR, and in practice carries a scale error that changes from frame to frame. The RL policy in Phase 3 is chosen partly because it is more robust to noisy depth than classical MPC-based foothold planning (OCS2/legged_perceptive), which requires precise terrain geometry.

**Stereo fallback decision:** NVIDIA Isaac ROS stereo stack (cuVSLAM, FoundationStereo) would be significantly richer if a second camera is added. This is an open decision; if the cost allows, stereo should be reconsidered before the Phase 2 hardware commitment.

### Modular, Not End-to-End
Each layer is independently replaceable. AI is injected only at the locomotion boundary (Phase 3 RL policy) and at the perception boundary (depth inference). Everything in between — scale estimation, elevation mapping, traversability, MPPI, EKF localization — is classical and fully debuggable without neural network involvement.

### Vehicle-Agnostic Above the Odometry Boundary
All platforms publish `nav_msgs/Odometry` + a traversability costmap. Elevation mapping, MPPI, and localization EKF are shared across vehicle types. Only the odometry source and drive controller differ per platform:

| Platform | Odometry | Drive Controller |
|---|---|---|
| Quadruped | LKO + ORB-SLAM3 EKF | Gait controller |
| Ackermann | Wheel encoder + GPS (or `lidar_odometry`) | ackermann_steering_controller |
| Tracked | Wheel encoder + IMU (or `lidar_odometry`) | Differential drive |
| USV | GPS + DVL | Thruster controller |

### Pure-numpy Cores, Thin ROS Nodes
The new modules keep their math in pure-numpy files that can be tested on the host (`quadruped_kinematics`, `gait.py`, `correction_models.py`, `trajectory_compare.py`). The ROS nodes are thin wrappers around them. Pluggable parts (depth backends, correction models, fit models) are registered by name and selected with a single parameter.

---

## Kinematics — Shared Package

**Implementation:** `src/quadruped_kinematics/` (pure numpy)

Single source of truth for Spot Micro geometry (`SPOT_MICRO`): hip pivots 0.186 m × 0.078 m, 55 mm hip link, 107.5 mm upper leg, 130 mm lower leg. Provides closed-form per-leg IK/FK/Jacobian and whole-body IK (body pose over fixed footholds). The gait controller and leg odometry both use it. Its tests parse the URDF xacro and fail if the numbers diverge. The joint order contract is `JOINT_ORDER`: `FL hip_roll, hip_pitch, knee, FR …, RL …, RR …`, which matches `ros2_control.yaml`.

---

## Locomotion — Gait Controller

**Implementation:** `src/quadruped_gait_controller/` — `gait_node.py` (ROS) + `gait.py` (pure-python `BezierGait`)

**Pipeline:** `/cmd_vel` → clamp, acceleration ramp, watchdog → `BezierGait` (footholds in the nominal body frame) → whole-body IK (`quadruped_kinematics`) → `/quadruped_joint_controller/commands` (12 positions) at 100 Hz. The node publishes joint commands only. `/odom_lko` comes from `quadruped_leg_odometry`.

**Gaits:**

| `gait_type` | Cycle | Duty | Swing | Body shift | Use |
|---|---|---|---|---|---|
| `trot` (default) | 0.4 s | 0.70 | 0.12 s, 20 mm lift | off | Everyday locomotion, 0.2–0.3 m/s |
| `walk` | 1.0 s | 0.75 | 0.25 s, 30 mm lift | into the support triangle | Slow, statically stable crawl |

- **Stance:** each foot sweeps linearly by the body's ground displacement at that foot (`v + ω × r`), so yaw becomes tangential strides. All legs share one scale factor when any stride exceeds `max_step_length` (0.10 m), so a turn stays a turn.
- **Swing:** 11th-order Bézier curve (Hyun et al. 2014, the MIT Cheetah / spot_mini_mini curve), tangent to the ground at both ends for soft touchdown.
- **Stopping:** a zero command finishes the current cycle before the robot holds a static stand.
- **Limits:** clamps `max_vx/vy/wz` = 0.4 m/s, 0.25 m/s, 1.2 rad/s; acceleration ramps 0.6 m/s², 2.5 rad/s². All tuning is ROS parameters (`config/gait_params.yaml`); per-gait defaults live in `GAIT_LIBRARY`.
- **Validated** headless in Gazebo: straight at 0.2–0.3 m/s with <2° roll/pitch, turns at the commanded rate. A duty-0.5 flight trot tips over under position control and should not be revisited.

**Phase 3 upgrade path:** Replace with an Isaac Lab RL policy exported to TensorRT. The policy receives `/cmd_vel` + `/elevation_map`. NVIDIA provides a documented Jetson Orin transfer path for this.

---

## Perception — Monocular Depth

**Implementation:** `src/monocular_depth/monocular_depth/depth_node.py`

**Model:** Metric3D V2, intrinsics-conditioned. Supports `metric3d_vit_small`, `metric3d_vit_large`, `metric3d_vit_giant2` via the `model_name` parameter. Default: ViT-S for the Jetson power budget. Shares the repo-root `compat/mmcv/` shim (an mmengine-backed stand-in for Metric3D's mmcv 1.x imports) with `depth_comparison`, so both images build from the repository root.

It replaced Depth Anything V2 after the Phase 1 side-by-side in `src/depth_comparison`. DA-V2's metric head ends in `sigmoid(x) * max_depth`, so its scale is inherited from a training domain. Metric3D instead takes its scale from the camera that took the picture. That does not fully solve the scale problem (see below).

**I/O:**
- Subscribes: RGB image + the matching `camera_info`. The CameraInfo is **required**: without it there is no metric scale, so frames are dropped until one arrives.
- Publishes: `/depth/image_raw` (metric depth, `32FC1`, metres) + `/depth/camera_info` (timestamp-synced)

**Rate limiting:** Configurable target FPS (default 10 Hz) is an upper bound. Metric3D measures ~1.2–1.5 s/frame at 640×480 on the dev host (RTX 2060), so the node drops what it cannot keep up with. Expect ~4× fewer point clouds than under DA-V2; the Jetson budget needs a TensorRT FP16 export, which is still to come.

**Device fallback:** If CUDA is requested but unavailable, falls back to CPU with a warning.

### Depth Model Comparison

**Implementation:** `src/depth_comparison/`

One node runs every configured backend on the **same** RGB frame and publishes `/depth_compare/<key>/{image_raw,camera_info,points}` per model with the source header verbatim, so all clouds overlay in RViz. Separate rate-limited nodes would drift onto different frames, which is why everything runs in one process.

| Key | Scale comes from | ms/frame (RTX 2060) |
|---|---|---|
| `gt` | Gazebo depth sensor (reference relay) | — |
| `dav2_vkitti` | VKITTI training domain (driving), 80 m | ~340 |
| `dav2_hypersim` | Hypersim training domain (indoor), 20 m | ~300 |
| `metric3d_s` | Live camera intrinsics | ~1170 |

A backend that fails to load is dropped, not fatal. New models implement `DepthBackend` and register in `backends/__init__.py`. Visual inspection in Gazebo pointed to Metric3D as the closest to ground truth. A quantitative comparison module is deferred until visual inspection proves insufficient.

---

## Depth Scale Problem — Kinetic Scale Probes

Working hypothesis: monocular depth error is a per-frame scale factor, `depth_pred ≈ s · depth_true`. Two independent probes estimate `s` at runtime without ground-truth depth. A third node applies the estimate.

### Alternative 1 — Ground Plane (`src/ground_plane_estimation/`)

PointCloud2 → TF to base frame → low-z candidate gate → RANSAC plane. Publishes `/ground_plane[/<key>]/{distance,camera_height,scale,marker}`.

- The camera-to-plane distance is the observable: the rigid TF only rotates the camera-relative cloud, so a scaled cloud gives a scaled camera height.
- `scale = measured camera height / expected camera height`. The expected height comes from a nominal standing height (`expected_base_height`) or per-frame from TF (`height_reference_frame=odom`), which is exact in Gazebo ground-truth mode because `odom` z=0 is the ground.
- One estimator per input topic (`GROUND_PLANE_INPUT_TOPICS`), so every depth comparison channel gets its own scale channel.
- Docs: `documentation/ground_plane_estimation.md`.

### Alternative 2 — Motion (`src/lidar_odometry/` + `src/odometry_scale_estimation/`)

KISS-ICP on a uniformly scaled cloud estimates a uniformly scaled motion. Comparing that motion against metric leg odometry recovers `s`.

- **`lidar_odometry`:** KISS-ICP (adaptive-threshold ICP, constant-velocity prior, voxel hash map) on any PointCloud2 → `/odom_lidar` (`odom`→`base_link`, drift-model covariance, no TF by default). Pure-LiDAR on purpose: it is sensor-agnostic and needs no IMU sync. It is also a drop-in odometry source for LiDAR-equipped platforms. `documentation/lidar_odometry.md` compares it against FAST-LIO2, Point-LIO, DLIO and LIO-SAM.
- **`odometry_scale_estimation`:** sliding window (default 5 s; 10 s in the probe profile) over a reference stream (`/odom_lko`) and one or more estimate streams. It fits matched displacement chords, which are frame-invariant, so no extrinsic alignment is needed and the reference is interpolated at estimate stamps. Publishes `/odom_scale[/<key>]/{scale,shift,residual,ref_motion,marker}`. Fit models: `scalar` (`chord_est = s·chord_ref`) or `affine` (`+ b`, which needs chord-length spread). The `min_motion` excitation gate suppresses output while standing, and `ref_motion` separates "standing" from "broken".
- **`affine` shift is a displacement-space diagnostic**, not the depth-space shift of the correction node's affine model.
- The probe profile runs two channels: `/odom_scale/odom_lidar/scale` (the measurement) and `/odom_scale/odom_ground_truth/scale` (Gazebo truth vs. the same reference, which should read 1.0; its deviation is the reference's noise floor).
- Docs: `documentation/odometry_scale_estimation.md`.

### Correction (`src/pointcloud_scale_correction/`)

Subscribes a PointCloud2 + a Float32 scale and republishes `<input>_corrected` with the **original header** (frame and stamp untouched), plus `<output>/applied_scale`.

- Pluggable error model (`correction_models.py`): `scalar` (`p / s`, default, any frame) or `affine` (`(z−b)/s` along each ray, camera optical frame required).
- Estimates are smoothed (`ema` / `median`) and gated (`min_scale` / `max_scale`).
- `stale_policy` (`hold` / `passthrough` / `drop`) handles gaps; before the first estimate, `hold` drops instead of passing wrong-scale clouds.
- One node per input via `SCALE_CORRECTION_INPUT_TOPICS`.

---

## Localization

**Implementation:** `src/localization/` (ORB-SLAM3 node + `config/ekf_local.yaml`, `ekf_global.yaml`) and `src/quadruped_leg_odometry/`

### Localization Modes (`LOCALIZATION_MODE`)

| Mode | `odom→base_link` | `map→odom` / map pose | Used by |
|---|---|---|---|
| `ground_truth` | Gazebo ground-truth odometry node | Static identity TF | Depth comparison and scale probe profiles |
| `slam` | Local EKF | Global EKF | Full Gazebo sim |
| `offline_video` | — | ORB-SLAM3 owns `map→base_link` directly; static camera TF from the launch file | Offline video pipeline |

Each TF edge has exactly one publisher in every mode.

### Leg Kinematics Odometry (`quadruped_leg_odometry`)

- **IMU attitude:** uses the IMU quaternion when present (Gazebo). Otherwise it falls back to a Mahony-style complementary filter for rate-only hardware (yaw then drifts).
- **Leg odometry:** FK + geometric Jacobians for all four legs from `/joint_states` via `quadruped_kinematics`. Stance feet are detected kinematically (height band above the lowest foot along gravity). Each stance foot is stationary, so `v_base = −(ω × p_foot + J q̇)`. Estimates are averaged and integrated with the IMU attitude.
- **Output:** `/odom_lko` (`odom`→`base_link`, twist in `base_link`), no TF by default.
- **Covariances are meaningful:** twist covariance is inflated ×100 with fewer than two stance feet, and pose covariance grows with distance and angle. Fuse the twist, not the pose.

### ORB-SLAM3 RGBD Node (C++)

Synchronizes RGB and depth (ApproximateTime) and runs ORB-SLAM3 tracking with Metric3D depth as the D channel. Publishes `/odom_slam`.

- Sim camera: 640×480, fx=fy=381.36, cx=320, cy=240, no distortion.
- ORB: 1000 features, scale factor 1.2, 8 levels.
- Converts the optical frame to REP-103 base_link.
- Suppresses output while tracking is LOST.

### EKF Fusion (robot_localization, dual filter)

| Filter | Rate | World frame | Inputs | Output |
|---|---|---|---|---|
| Local | 50 Hz | `odom` | `/odom_ground_truth` (temporary), IMU | `/odom` + TF `odom→base_link` |
| Global | 30 Hz | `map` | `/odom` + `/odom_slam` | `/odometry/global` + TF `map→odom` |

**Open item:** the local EKF still fuses Gazebo ground truth. The `/odom_lko` block (body-frame twist only) is written and commented out in `ekf_local.yaml`, ready to switch now that leg odometry exists.

---

## Planning — Nav2 + MPPI

**Implementation:** `src/nav2_mppi_navigation/nav2_mppi_navigation/config/nav2_mppi_params.yaml` (see also `documentation/mppi_controller.md`)

### MPPI Controller Configuration

| Parameter | Value | Note |
|---|---|---|
| `batch_size` | 2000 | Rollout samples per step |
| `time_steps` | 56 | Horizon steps |
| `model_dt` | 0.05 s | 2.8 s horizon |
| `motion_model` | `DiffDrive` | Should be `Omni` for lateral quadruped motion |
| `vx_max` / `vx_min` | 0.5 / −0.35 m/s | Above the gait controller's 0.4 m/s clamp |
| `vy_max` | 0.0 | Set 0.5 (≤ gait's 0.25 clamp in practice) when switching to Omni |
| `wz_max` | 1.9 rad/s | Above the gait controller's 1.2 rad/s clamp |
| `temperature` | 0.3 | MPPI smoothing |
| Controller frequency | 20 Hz | |

**Critics enabled:** ConstraintCritic, CostCritic (footprint-aware, collision_cost=1e6), GoalCritic, GoalAngleCritic, PathAlignCritic (weight 14.0), PathFollowCritic, PathAngleCritic, PreferForwardCritic.

### Costmaps

- **Local:** 4m×4m rolling window, 5cm resolution, StaticLayer on the elevation-mapping occupancy grid
- **Global:** 15m×15m rolling window, 5cm resolution, same source
- **Footprint:** 0.40m × 0.30m rectangle, inflation radius 0.55m

### Global Planner

NavFn (Dijkstra) — sufficient for Phase 1/2. SmacPlannerLattice is the intended upgrade for terrain-aware global paths.

---

## Mapping — Elevation Mapping CuPy

GPU-accelerated elevation map using CuPy CUDA kernels (vendored ANYbotics `elevation_mapping_cupy`, not modified in place). Ingests `PointCloud2` from depth_image_proc and publishes `/elevation_map` (`grid_map_msgs/GridMap`) and an `OccupancyGrid`.

**Parameters:** 3m×3m, 5cm resolution, 2 Hz update. An expansion to 10m×10m is planned for open terrain, subject to Orin Nano GPU memory.

**Fixed frame:** `map` in offline-video mode (ORB-SLAM3 owns `map→base_link`), `odom` in Gazebo with the dual EKF (loop-closure jumps in `map` would corrupt accumulated clouds).

**Traversability:** `traversability_estimation` (slope angle, height variance, step height vs. leg reach → `/traversability_costmap`) is designed but not yet implemented. Nav2 currently consumes the raw elevation-mapping occupancy grid.

---

## Offline Testing — Video Camera Simulator

**Implementation:** `src/video_camera_sim/video_camera_sim/video_node.py`

Plays pre-recorded video files as a live camera feed, so the monocular depth and ORB-SLAM3 pipelines can be developed without Gazebo or a physical robot.

- Reads MP4/AVI/MKV/MOV via OpenCV and publishes `/camera/image_raw` (BGR8) + `/camera/camera_info` with synchronized stamps
- Configurable FPS, resolution and looping, with runtime parameter updates
- `offline-video.env` is the single source of truth: `VIDEO_FPS=10`, `VIDEO_LOOP=false` (looping corrupts the SLAM map), `VIDEO_START_DELAY=20` s for model loading
- **`FX`/`FY`/`CX`/`CY` must be the recording camera's real calibration.** Metric3D derives metric scale from `fx`, so a wrong value scales all depth by the same factor.

Next test bench after Gazebo: the same depth models and scale probes on pre-recorded iPhone footage. **Gazebo cannot validate the perception stack**: its depth is ground truth, and real camera effects (auto-exposure, rolling shutter, motion blur) only show up in recorded footage.

---

## Robot Model (URDF)

**File:** `src/quadruped_gazebo/urdf/quadruped.urdf.xacro`

**Body:** Spot Micro — 200mm × 90mm × 60mm chassis plus head and tail, 1.3 kg (2.82 kg total). Hip pivots 186mm apart fore/aft, 78mm apart left/right. Stand height 160mm; the stand pose (`initial_value`) is derived from the geometry, so the robot spawns standing.

**Legs (×4, mirrored, all knees backward):**
- Hip abduction link: 55mm, 0.15 kg
- Upper leg: 107.5mm, 0.16 kg
- Lower leg: 130mm, 0.05 kg
- Foot sphere: 15mm radius, 0.02 kg
- Servo limits: 3.4 N·m, 7 rad/s

**Joints:** 12 total (hip_roll, hip_pitch, knee per leg). All controlled via `position_controllers/JointGroupPositionController` through `ros2_control`. Link lengths are tested against `quadruped_kinematics`.

**Sensors (sim):**
- Front RGB camera: 640×480, hfov=80°, fx=fy=381.36
- Depth camera: same intrinsics, publishes depth image + PointCloud2
- IMU: standard Gazebo plugin, co-located with `base_link`

---

## Simulation Infrastructure

**Simulator:** Gazebo Harmonic (gz-sim), interfaced via `ros_gz_bridge` and `ros_gz_sim`. `gazebo.launch.py headless:=true` runs the server only (CI, no display).

**Launch sequence** (`gazebo.launch.py`):
1. Robot State Publisher (URDF → `/robot_description`)
2. Gazebo Harmonic with SDF world
3. Spawn robot via `ros_gz_sim create`
4. After spawn: load `joint_state_broadcaster` → then `quadruped_joint_controller`
5. Start the ROS-Gazebo topic bridge
6. `gazebo_ground_truth_odometry.py` → `/odom_ground_truth`, and additionally TF `odom→base_link` in `ground_truth` mode
7. Static `map→odom` in `ground_truth` mode only (the global EKF owns it in `slam` mode)

**Simulation benchmark** (`simulation_benchmark.launch.py`, `documentation/simulation_benchmark.md`): `v0_geometry` is a deterministic, procedurally generated outdoor terrain from `config/benchmarks/v0_geometry.yaml` (seed 42) via `generate_benchmark_world.py`. It has slopes, rough ground, rock fields, rubble, a ditch, a narrow bridge and a constrained passage. It is a geometry and integration benchmark, not a monocular depth validation.

**tmux session management:** `tmux-attach.sh` creates one tmux session per container, each pre-configured with the correct `docker exec` command.

---

## Key Software Stack

| Layer | Technology |
|---|---|
| OS | Ubuntu 24.04 (Jetson) / Ubuntu 24.04 (dev host) |
| Robot middleware | ROS 2 Jazzy |
| Simulation | Gazebo Harmonic |
| DDS | CycloneDDS (`rmw_cyclonedds_cpp`) |
| Languages | Python 3, C++ (ROS 2 nodes) |
| GPU inference | PyTorch (dev), TensorRT FP16 (deployment, planned) |
| Depth model | Metric3D V2 (production); DA-V2 VKITTI/Hypersim (comparison only) |
| Scale estimation | RANSAC ground plane; KISS-ICP + chord-matching vs. leg odometry |
| SLAM | ORB-SLAM3 (RGBD mode) |
| Sensor fusion | robot_localization dual EKF |
| Navigation | Nav2 (MPPI local planner, NavFn global planner) |
| Elevation mapping | elevation_mapping_cupy (CuPy CUDA kernels) |
| Container runtime | Docker + Docker Compose v2 (`include:` federation) |

---

## Known Limitations & Open Decisions

| Item | Status | Detail |
|---|---|---|
| Monocular depth scale | Active work | Per-frame scale error persists with Metric3D. Two kinetic probes plus correction are implemented; they still need validation in Gazebo, then on recorded footage. Whether the error is scalar or affine is still open. |
| Scale probe floors | Design tradeoff | Alt. 1 assumes a visible, roughly planar ground and a known camera height. Alt. 2 needs translation (pure rotation is unobservable), and LKO slip reads as scale change. The ground-truth channel measures the reference noise floor. |
| Metric3D throughput | Performance | ~0.7 Hz on the dev GPU. TensorRT FP16 export for Orin is not done yet. KISS-ICP registration across frames this sparse is marginal, so drive slowly in the odometry probe. |
| Local EKF input | Open item | Still fuses `/odom_ground_truth`; switch to `/odom_lko` (block prepared in `ekf_local.yaml`). |
| Traversability estimation | Not implemented | Nav2 consumes the raw elevation-map occupancy grid. |
| MPPI motion model | Open item | Currently `DiffDrive`; must be `Omni` for lateral quadruped motion. `vy_max` = 0 as a guard. Velocity limits exceed the gait controller's clamps. |
| ORB-SLAM3 depth input | Design tradeoff | Uses monocular depth as the D channel, not a true stereo pair. Metric accuracy depends on the depth network's scale and the camera calibration given to Metric3D. Requires real-hardware validation. |
| Stereo camera option | Open decision | Isaac ROS stereo stack (cuVSLAM, FoundationStereo) would be richer. Reconsider if the second camera's cost is acceptable before the Phase 2 hardware commitment. |
| Costmap resolution | Scaling concern | 3m×3m at 5cm is right for controlled outdoor. Expanding to 10m×10m needs a GPU memory audit on Orin Nano. |
| QGroundControl integration | Planned | Requires a MAVLink bridge (MAVROS or px4_ros_com) publishing waypoints as `nav2_msgs/action/NavigateThroughPoses`. |
| Phase 3 RL policy training | Future | Requires an Isaac Lab setup. Target: zero-shot transfer to Jetson Orin via TensorRT FP16 export. |

---

## Environment Variables

| Variable | Default | Purpose |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | Isolate ROS 2 DDS domain |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS middleware selection |
| `DISPLAY` / `XAUTHORITY` | from host | Gazebo, RViz, ORB-SLAM3 viewer via X11 |
| `NVIDIA_VISIBLE_DEVICES` | `all` | GPU access in CUDA containers |
| `CUDA_VISIBLE_DEVICES` | `0` | Single GPU, avoids allocation conflicts |
| `USE_SIM_TIME` | profile-dependent | `true` in Gazebo profiles, `false` offline |
| `LOCALIZATION_MODE` | profile-dependent | `ground_truth` / `slam` / `offline_video` |

Module-specific variables (`GROUND_PLANE_*`, `LIDAR_*`, `ODOM_SCALE_*`, `SCALE_CORRECTION_*`, `COMPARE_*`, `VIDEO_*`) are documented in each profile's env file and module README.

---

## Adding a New Module

1. Create `src/<module>/` with `package.xml`, `docker-compose.yaml`, `CMakeLists.txt` or `setup.py`
2. Choose base image: `dustynv/ros:jazzy-ros-base-l4t-r36.4.0` (CPU) or `dustynv/ros:jazzy-pytorch-l4t-r36.4.0` (GPU/inference)
3. Add `include: - path: src/<module>/docker-compose.yaml` to root `docker-compose.yaml`
4. Add the container name to the `CONTAINERS` array in `tmux-attach.sh`
5. Add a launch smoke test: node starts, required topics appear within 10 seconds
