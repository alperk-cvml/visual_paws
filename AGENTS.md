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

---

## Reference Material

For detailed technical specifications, agents must always cross-reference `CLAUDE.md`, which contains the authoritative source of truth for:
- Development Phases and Current State
- The Full Architecture Pipeline and Container Layout
- ROS 2 Topic Flow and the Vehicle Interface Contract
- Key Source Files
- Localization and Locomotion Strategies
- Procedures for Adding New Modules

Failure to align with the constraints listed in `CLAUDE.md` and this `AGENTS.md` file will result in broken container builds, incompatible ROS 2 configurations, or un-deployable code on the Jetson target.



## Current Target

Currently my target is to create an elevation map using offline recorded video. Using pose from ORB-SLAM3 and points from DepthAnythingV2.

Step 0: Publish video from a file - done
Step 1: Get monodepth working - done
Step 2: Get pointclouds published from monodepth - done
Step 3: Get SLAM working - done
Step 4: Integrate SLAM TF with pointclouds from monodepth - we are herr

For this step we need to fix the TF tree.
