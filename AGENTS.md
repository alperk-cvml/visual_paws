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
2. Be mindful of user and group permissions when interacting with mounted Docker volumes to avoid `Permission Denied` errors.

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
- [ ] Step 1: DAV2 VKITTI
- [ ] Step 2: DAV2 Hypersim
- [ ] Step 3: Metric3D-v2-S

I'll thest these in two phases:
- [] Phase 1: ROS2 Gazebo <-- We are here 
- [] Phase 2: On pre-recorded (iphone) videos 

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

This sets `USE_SIM_TIME=true`, `LOCALIZATION_MODE=ground_truth` (static `map→odom` plus `odom→base_link`; no EKF or ORB-SLAM3, since the comparison needs a TF tree rather than drift correction), and `COMPARE_BACKENDS=gt,dav2_vkitti,dav2_hypersim,metric3d_s`. `depth-comparison.env` is the single source of truth for this profile.

The module lives in `src/depth_comparison/`. Constraints agents must preserve:

- **One node runs every backend on the same frame.** Do not split the models into separate nodes — independent rate limiters drift onto different source frames, and comparing depth maps of different moments measures nothing.
- **Depth images carry the source image header verbatim.** Never restamp or reframe them; the per-model point clouds must stay in the camera optical frame with the source timestamp.
- **`max_depth` is a trained-in output scale for Depth Anything V2**, not a clamp — its metric head ends in `sigmoid(x) * max_depth`. Raising Hypersim's 20 m for an outdoor scene multiplies every prediction by 4 and manufactures a scale error.
- **`src/depth_comparison/compat/mmcv/` is a deliberate compatibility shim**, not dead code. Metric3D imports mmcv 1.x, which will not install on Python 3.12; mmcv 2.x removed the `mmcv.utils` namespace. The shim re-exports the four symbols Metric3D needs from mmengine and raises on anything else. Extend it rather than patching the Metric3D checkout in the torch hub cache.
- **Add new models by implementing `DepthBackend`** and registering the class in `backends/__init__.py`, then adding a channel to `config/backends.yaml`. The node declares the backend's parameters from `param_defaults()` and the launch file spawns the matching point cloud automatically.