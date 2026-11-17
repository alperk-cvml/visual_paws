# AGENTS.md

This file provides guidance to autonomous AI agents (e.g., Antigravity, Gemini, Copilot) when working with code in the Visual Paws repository. It is designed to work in tandem with `CLAUDE.md`.

## Project Overview

**Visual Paws** is a ROS 2 quadruped robot autonomous navigation stack targeting real outdoor deployment on a 12-DOF quadruped via **camera-only perception**. It uses ROS 2 Jazzy, Gazebo Harmonic, and Docker on NVIDIA Jetson (ARM64).

Agents must understand that this repository is heavily containerized and relies on specific ROS 2 topics and TF trees instead of monolithic execution.

---

## Agent Directives

### 1. Code Modification & Tools
- **Use the right tools:** Prioritize targeted API tools (e.g., `replace_file_content`, `grep_search`, `view_file`) over generic shell commands (`cat`, `sed`, `grep`).
- **No bare-metal assumptions:** All ROS 2 packages are designed to run inside Docker containers. Do not assume host-level access to ROS 2 packages or bare-metal setups.
- **Preserve Documentation:** Keep existing comments and docstrings intact unless explicitly instructed to modify them.
- **No Placeholders:** Write fully functional code. Avoid using placeholder comments like `// ... existing code ...` unless utilizing chunk-replacement tools.

### 2. Architecture Awareness
- **Modular Design:** AI is injected primarily at the locomotion layer (Phase 3 RL policy). Do not suggest end-to-end AI overhauls for mapping or routing unless specifically requested by the user.
- **Hardware Constraints:** The target is NVIDIA Jetson Orin Nano/NX. Avoid adding dependencies that require x86-specific binaries or libraries that lack CUDA/ARM64 support.
- **Camera-Only Constraints:** Do not implement or suggest LiDAR packages (e.g., `velodyne`, `ouster`). Metric depth and surface normals are inferred exclusively from RGB via neural networks.

### 3. Build & Test Procedures
When instructed to compile, build, or test code, agents must:
1. Ensure the build occurs within the correct ROS 2 workspace using `colcon build --symlink-install`.
2. Be mindful of user and group permissions when interacting with mounted Docker volumes to avoid `Permission Denied` errors. Every module's container bind-mounts the same repo root to `/work/ws` and builds into the same shared `log/`/`build/`/`install/` dirs, so **every Dockerfile must end with a non-root `USER` at UID/GID 1000** (see `CLAUDE.md` → "Non-Root Container User (required)" for the exact boilerplate). A single root-running container poisons that shared mount's ownership for every other module — this is not a style preference, it is the direct cause of `PermissionError` build failures. When creating or editing any module's Dockerfile, verify it ends with `USER ${USERNAME}` before considering the task done.

### 4. Vendored Code — Do Not Modify
- `src/elevation_mapping_cupy_pkg/elevation_mapping_cupy/` (upstream ANYbotics repo) and `src/localization/ORB_SLAM3/` (upstream ORB-SLAM3) are vendored third-party code. Never edit files in these directories — changes are lost on update and make upstream fixes unmergeable.
- To change behaviour, use configuration/parameters, a wrapper node, or a documented patch file in `patches/`. If a change genuinely requires modifying upstream, say so explicitly and propose it as a patch rather than editing silently.

### 5. TF Frames & Conventions (REP-105 / REP-103)
Non-compliant frame names or TF tree structures are bugs, not style issues.
- **Never invent frame names** (`robot`, `world`, `camera_frame`, `base`, `map_frame`). Use only the canonical set: `map`, `odom`, `base_link`, `base_footprint`, `camera_link`, `camera_optical`, `imu_link`. New sensor frames follow `<sensor>_link` / `<sensor>_optical`.
- **Each TF edge has exactly one publisher.** Adding a new publisher for an edge means removing the old one — never leave two nodes publishing the same transform.
- ORB-SLAM3 publishes `map→odom`, never `map→base_link` directly (except in `offline_video` mode, where it owns `map→base_link`). Static transforms come only from `robot_state_publisher` or `static_transform_publisher`.
- **Point clouds from `depth_image_proc` are in `camera_link_optical` and carry the source image timestamp — never restamp or reframe them.**
- Units/axes follow REP-103: x forward, y left, z up; radians; metres/seconds/radians only, never mm or degrees.

---

## Reference Material

For detailed technical specifications, agents must always cross-reference `CLAUDE.md`, which contains the authoritative source of truth for:
- Development Phases and Current State
- The Full Architecture Pipeline and Container Layout
- ROS 2 Topic Flow and the Vehicle Interface Contract
- ROS 2 Conventions & TF Standards (REP-105/REP-103, authoritative TF tree)
- Offline Video Workflow (pipeline stages, Gazebo-vs-offline differences, `offline-video.env` rules)
- Sim vs. Real Topic Names (never hardcode either; must be remappable via launch args)
- Key Source Files
- Localization and Locomotion Strategies
- Procedures for Adding New Modules
- Vendored Code boundaries (do not modify upstream-vendored packages)

Failure to align with the constraints listed in `CLAUDE.md` and this `AGENTS.md` file will result in broken container builds, incompatible ROS 2 configurations, or un-deployable code on the Jetson target.



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

### Offline Video Launch

The offline pipeline is launched with an env-file override:

```bash
docker compose --env-file offline-video.env up -d --build
```

This sets `LOCALIZATION_MODE=offline_video` (ORB-SLAM3 directly publishes `map→base_link`), `USE_SIM_TIME=false`, and wires all topic remappings. Without the env file, containers fall back to Gazebo simulation defaults.

**Important**: Agents must ensure `offline-video.env` variables are respected when modifying docker-compose files or launch parameters. The env file is the single source of truth for the offline pipeline configuration.

### Depth Comparison Launch (Phase 1)

The Phase 1 comparison runs against Gazebo, whose depth sensor supplies the ground-truth reference channel that recorded footage cannot:

```bash
docker compose --env-file depth-comparison.env \
  -f docker-compose-depth-comparison.yaml up -d --build
```

This sets `USE_SIM_TIME=true`, `LOCALIZATION_MODE=ground_truth` (static `map→odom` plus `odom→base_link`; no EKF or ORB-SLAM3, since the comparison needs a TF tree rather than drift correction), selects the deterministic `v0_geometry` simulation benchmark, and sets `COMPARE_BACKENDS=gt,dav2_vkitti,dav2_hypersim,metric3d_s`. `SIM_BENCHMARK_SEED=config` uses `random_seed: 42` from the scenario YAML; set an explicit integer for another repeatable terrain. `depth-comparison.env` is the single source of truth for this profile's runtime selection and overrides.

The module lives in `src/depth_comparison/`. Constraints agents must preserve:

- **One node runs every backend on the same frame.** Do not split the models into separate nodes — independent rate limiters drift onto different source frames, and comparing depth maps of different moments measures nothing.
- **Depth images carry the source image header verbatim.** Never restamp or reframe them; the per-model point clouds must stay in the camera optical frame with the source timestamp.
- **`max_depth` is a trained-in output scale for Depth Anything V2**, not a clamp — its metric head ends in `sigmoid(x) * max_depth`. Raising Hypersim's 20 m for an outdoor scene multiplies every prediction by 4 and manufactures a scale error.
- **`compat/mmcv/` (repository root) is a deliberate compatibility shim**, not dead code. Both `monocular_depth` and `depth_comparison` COPY it, which is why those two images alone build with the repository root as their Docker context (`dockerfile: src/<module>/Dockerfile`) instead of their own directory. Metric3D imports mmcv 1.x, which will not install on Python 3.12; mmcv 2.x removed the `mmcv.utils` namespace. The shim re-exports the four symbols Metric3D needs from mmengine and raises on anything else. Extend it rather than patching the Metric3D checkout in the torch hub cache.
- **Add new models by implementing `DepthBackend`** and registering the class in `backends/__init__.py`, then adding a channel to `config/backends.yaml`. The node declares the backend's parameters from `param_defaults()` and the launch file spawns the matching point cloud automatically.

### Depth Scale Probe Launch (Alternative 1)

Live version of the "solve it via ground plane distance" alternative above, run against the production `monocular_depth` node (Metric3D V2 only) rather than the `depth_comparison` harness:

```bash
docker compose --env-file depth-scale-probe.env \
  -f docker-compose-depth-scale-probe.yaml up -d --build
```

Same `LOCALIZATION_MODE=ground_truth` + `v0_geometry` benchmark as the depth comparison profile. `monocular_depth` runs on `/front_camera/image_raw`; `depth_image_proc` bridges its `/depth/image_raw` output into `/depth/points` (`ground_plane_estimation` needs a point cloud, not a depth image — this is the one module added beyond a literal reading of the module list); `ground_plane_estimation` fits the ground plane in that cloud with `GROUND_PLANE_HEIGHT_REFERENCE_FRAME=odom`, so `/ground_plane/scale` is a live per-frame depth-scale estimate. `depth-scale-probe.env` is the single source of truth for this profile.

The module lives in `src/ground_plane_estimation/`. Constraints agents must preserve:

- **`/ground_plane/camera_height` is the scale observable; `/ground_plane/distance` is not.** The camera-origin-to-plane distance scales cleanly with the depth model's scale error because only rotation (not the un-scaled TF translation) separates it from the raw predicted cloud. `/ground_plane/distance` (measured from `base_link`) mixes in that translation and is for plotting only — see `documentation/ground_plane_estimation.md` for the derivation.
- **`height_reference_frame` (e.g. `odom`) is only a valid ground reference under `LOCALIZATION_MODE=ground_truth`.** It assumes that frame's z=0 plane is the true, driftless ground. Do not reuse it for SLAM or offline-video profiles; use `expected_base_height` there instead.
- **The candidate-point gate is percentile-based, not an absolute height cutoff.** An absolute threshold silently breaks once the depth model's scale error moves the cloud's apparent height — the whole point of this module is to work without assuming the scale is already known.

### Odometry Scale Probe Launch (Alternative 2)

Live version of the "use IMU and leg odometry to estimate the motion and
compare it to the motion inferred with the monodepth pointcloud" alternative:

```bash
docker compose --env-file odometry-scale-probe.env \
  -f docker-compose-odometry-scale-probe.yaml up -d --build
tmux attach -t quadruped_test_controller   # the probe needs motion
```

Same `LOCALIZATION_MODE=ground_truth` + `v0_geometry` benchmark and the same
`monocular_depth` → `depth_image_proc` front end as Alternative 1. From
there: `lidar_odometry` runs KISS-ICP on `/depth/points` (so `/odom_lidar`
inherits the monocular cloud's scale error), `quadruped_leg_odometry`
supplies the metric reference `/odom_lko`, and `odometry_scale_estimation`
compares the two over a sliding window. `pointcloud_scale_correction` applies
the result to `/depth/points_corrected`. `odometry-scale-probe.env` is the
single source of truth for this profile.

The module lives in `src/odometry_scale_estimation/`. Constraints agents must
preserve:

- **The gait controller does NOT publish `/odom_lko`** — `quadruped_leg_odometry`
  does. Dropping that service from the profile leaves the probe permanently
  silent with no error, because its reference stream never arrives.
- **Chord lengths are the observable, not poses.** The two odometries have
  unrelated frame origins; comparing positions directly would require an
  extrinsic estimate. Chord length is frame-invariant, so only time alignment
  is needed. Do not "fix" this by aligning the trajectories and differencing
  poses.
- **The trajectory overlay marker aligns with rotation + translation only,
  never scale.** A scaled Kabsch/Umeyama fit would absorb exactly the quantity
  being measured and make the visual comparison useless.
- **Silence is a valid state.** The probe publishes nothing without motion
  (`min_motion`), without chord spread in affine mode, or across an odometry
  restart. `ref_motion` is published every cycle so "standing" stays
  distinguishable from "broken" — do not paper over the gates with a fallback
  estimate.
- **`/odom_scale/shift` (affine mode) is a displacement-space diagnostic**,
  not the depth-space `b` of `pointcloud_scale_correction`'s affine model.
  Wiring it into `SCALE_CORRECTION_SHIFT_TOPICS` would divide the cloud by a
  number measured in the wrong space.
