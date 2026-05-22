# Visual Paws

Modular ROS 2 quadruped simulation and control stack.

## Getting Started

### Prerequisites

- **Docker & Docker Compose** (v2.20+ required for `include` support)
- **tmux** (for managing multiple container terminals)
- **NVIDIA Container Toolkit** (for hardware-accelerated Gazebo)

### 1. X11 Authorization (Gazebo GUI)

To allow Docker to display the Gazebo GUI, run the following commands:

```bash
xhost +local:docker
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
```

### 2. Launch Modules

Build and start all modules (Gazebo, Point Cloud processing, Elevation Mapping, and Controllers) in the background:

```bash
docker compose up -d --build
```

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

## Development

The root `docker-compose.yaml` uses the `include` directive to pull in configurations from individual module directories under `src/`. This allows you to work on modules in isolation while maintaining a single orchestration point.

### Adding New Modules

1. Create your module directory in `src/`.
2. Add a `docker-compose.yaml` to your module.
3. Update the root `docker-compose.yaml` `include` section.
4. Add the container name to the `CONTAINERS` array in `tmux-attach.sh`.
