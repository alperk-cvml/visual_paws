# Visual Paws

Modular ROS 2 quadruped autonomous navigation stack using **camera-only perception**. Targets real outdoor deployment on a 12-DOF quadruped via monocular depth estimation (Depth Anything V2), visual SLAM (ORB-SLAM3), GPU elevation mapping, and traversability-aware planning.

Built on ROS 2 Jazzy · Gazebo Harmonic · CycloneDDS · Docker · NVIDIA CUDA.

## Getting Started

### Prerequisites

- **Docker & Docker Compose** (v2.20+ required for `include` support)
- **tmux** (for managing multiple container terminals)
- **NVIDIA Container Toolkit** (for hardware-accelerated Gazebo)

### 1. X11 Authorization (GUI)

To allow Docker to display GUIs (Gazebo, RViz, ORB-SLAM3 viewer), run:

```bash
xhost +local:docker
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
```

### 2. Launch Modules

**Gazebo simulation** (full sim pipeline):
```bash
docker compose up -d --build
```

**Offline video pipeline** (recorded video → monocular depth → ORB-SLAM3 → elevation map):
```bash
docker compose --env-file offline-video.env up -d --build
```

The `offline-video.env` profile configures:
- `video_camera_sim` to play a recorded video file as `/camera/image_raw`
- `monocular_depth` to infer metric depth via Depth Anything V2
- `localization` in `offline_video` mode (ORB-SLAM3 owns `map→base_link` directly)
- `elevation_mapping_cupy` to accumulate point clouds into a GPU elevation map

> **Note**: Place your video file at `videos/INPUT.MOV` (relative to repo root), or override `VIDEO_PATH` in `offline-video.env`.

### 3. Monitor & Interact (tmux)

Once the containers are running, use the provided `tmux-attach.sh` script to open a distinct tmux session for each module.

```bash
./tmux-attach.sh
```

- By default, it attaches to the `quadruped_gazebo` session.
- Use `Ctrl+b, d` to detach from a session.
- List all sessions: `tmux ls`
- Attach to a specific module, e.g., the keyboard controller:
  ```bash
  tmux attach -t quadruped_test_controller
  ```

## Architecture

See [CLAUDE.md](CLAUDE.md) for the full architecture pipeline, TF tree, topic flow, and design principles.

### Key Containers

| Container | Role |
|---|---|
| `quadruped_gazebo` | Physics sim, robot spawn, sensor publishing |
| `video_camera_sim` | Plays recorded video as camera topics (offline pipeline) |
| `monocular_depth` | Depth Anything V2 → metric depth from RGB |
| `depth_image_proc` | Depth image → PointCloud2 conversion |
| `localization` | ORB-SLAM3 + EKF sensor fusion |
| `elevation_mapping_cupy` | GPU elevation mapping from point clouds |
| `rviz` | Standalone RViz2 visualization |
| `nav2_mppi_navigation` | Nav2 + MPPI path planning |
| `quadruped_gait_controller` | Diagonal trot gait with analytic IK |
| `quadruped_test_controller` | Keyboard teleop |

## Development

The root `docker-compose.yaml` uses the `include` directive to pull in configurations from individual module directories under `src/`. This allows you to work on modules in isolation while maintaining a single orchestration point.

Environment variables are used to switch between sim and offline profiles. All topic names and frame IDs are remappable — see each module's `docker-compose.yaml` for available overrides.

### Adding New Modules

1. Create your module directory in `src/`.
2. Add a `docker-compose.yaml` to your module.
3. Update the root `docker-compose.yaml` `include` section.
4. Add the container name to the `CONTAINERS` array in `tmux-attach.sh`.
