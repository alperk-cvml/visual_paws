# quadruped_test_controller

A **temporary / test** ROS 2 package that lets you drive the simulated quadruped robot
with the **W / A / S / D** keyboard keys.  
It publishes `geometry_msgs/Twist` messages to the `/cmd_vel` topic.

> **Note** — This is a *dummy* controller for quick manual testing only.  
> It is not the production controller.

---

## Key Bindings

| Key | Action |
|-----|--------|
| `W` | Move forward |
| `S` | Move backward |
| `A` | Strafe left |
| `D` | Strafe right |
| `Q` / `ESC` | Quit |

Holding a key moves the robot; releasing it sends a **zero velocity** command (stops the robot).

---

## Running with Docker (recommended)

Make sure the **quadruped_gazebo** simulation is already running and sharing the same
`ROS_DOMAIN_ID` before starting the controller.

### 1. Build and start the container

```bash
cd src/quadruped_test_controller
# Start detached so Compose doesn't intercept the 'W' key!
docker compose up --build -d

# Attach directly to the container to send keystrokes
docker attach quadruped_test_controller
```

> The terminal where you ran `docker attach` is now the **keyboard input window**.  
> Focus it and press `W / A / S / D` to move the robot.

### 2. Verify messages (optional)

In a separate terminal on the host (or inside any other ROS container):

```bash
ros2 topic echo /cmd_vel
```

You should see non-zero `linear.x` / `linear.y` values while a key is held, and
zeros immediately after release.

### 3. Stop

Press `Q`, `ESC`, or `Ctrl+C` in the attached terminal, then:

```bash
docker compose down
```

---

## Running without Docker

If you have ROS 2 Jazzy installed locally:

```bash
# Install pynput
pip3 install pynput

# Build
cd /path/to/visual_paws   # workspace root (contains src/)
colcon build --symlink-install --packages-select quadruped_test_controller
source install/setup.bash

# Launch
ros2 launch quadruped_test_controller controller.launch.py
```

---

## Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ROS_DOMAIN_ID` | `0` | Must match the simulation container |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS middleware |

Set them as environment variables before running docker compose:

```bash
ROS_DOMAIN_ID=5 docker compose up
```

---

## Package Structure

```
quadruped_test_controller/
├── Dockerfile
├── docker-compose.yaml
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── quadruped_test_controller   # ament index marker
├── launch/
│   └── controller.launch.py
└── quadruped_test_controller/
    ├── __init__.py
    └── keyboard_controller_node.py
```
