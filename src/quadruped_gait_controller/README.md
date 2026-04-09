# quadruped_gait_controller

A ROS 2 Python package that converts `/cmd_vel` velocity commands into a **diagonal trot gait**,
computing joint angles via analytic leg IK and publishing them as
`trajectory_msgs/JointTrajectory` to the `ros2_control` joint trajectory controller.

> This is the **gait layer** — it sits between the keyboard controller and the simulator.  
> The keyboard controller sends `/cmd_vel`; this node translates that into joint motion.

---

## Architecture

```
quadruped_test_controller           quadruped_gait_controller          quadruped_gazebo
  (keyboard node)                      (this package)                    (simulation)
        │                                    │                                │
        │──── /cmd_vel (Twist) ─────────────►│                                │
        │                                    │── diagonal trot IK ──          │
        │                                    │──► /joint_trajectory_controller/joint_trajectory ──► │
```

## Gait Details

| Parameter | Value |
|---|---|
| Gait type | Diagonal trot |
| Stride period | 0.4 s |
| Step height | 3 cm |
| Max step length | 4 cm |
| Control rate | 80 ms |
| IK | Analytic 3-DOF (hip_roll, hip_pitch, knee) |

**Diagonal pairs:**
- Phase A (0–50%): FL + RR swing, FR + RL stance
- Phase B (50–100%): FR + RL swing, FL + RR stance

When `/cmd_vel` is zero the robot holds the standing pose and the gait phase resets.

---

## Running with Docker

Start the containers in this order:

```bash
# 1. Simulation
cd src/quadruped_gazebo
docker compose up -d

# 2. Gait controller (this package)
cd src/quadruped_gait_controller
docker compose up -d

# 3. Keyboard controller
cd src/quadruped_test_controller
docker compose up -d
docker attach quadruped_test_controller   # ← focus here and press W/A/S/D
```

### Stop all

```bash
docker compose -f src/quadruped_gazebo/docker-compose.yaml down
docker compose -f src/quadruped_gait_controller/docker-compose.yaml down
docker compose -f src/quadruped_test_controller/docker-compose.yaml down
```

---

## Running without Docker

```bash
pip3 install --break-system-packages setuptools
cd /path/to/visual_paws   # workspace root
colcon build --symlink-install --packages-select quadruped_gait_controller
source install/setup.bash
ros2 launch quadruped_gait_controller gait.launch.py
```

---

## Tuning

All gait parameters are constants at the top of
[`gait_node.py`](quadruped_gait_controller/gait_node.py):

| Constant | Default | Description |
|---|---|---|
| `STRIDE_PERIOD` | 0.4 s | Full stride duration |
| `STEP_HEIGHT` | 0.03 m | Foot lift during swing |
| `MAX_STEP_LENGTH` | 0.04 m | Stride length at max cmd_vel |
| `MAX_LATERAL` | 0.02 m | Step size for lateral cmd_vel |
| `TRAJ_DT_MS` | 80 ms | JTC publish interval |
| `STAND_HIP_PITCH` | 0.5 rad | Default standing hip pitch |
| `STAND_KNEE` | -1.2 rad | Default standing knee |

---

## Package Structure

```
quadruped_gait_controller/
├── Dockerfile
├── docker-compose.yaml
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── quadruped_gait_controller
├── launch/
│   └── gait.launch.py
└── quadruped_gait_controller/
    ├── __init__.py
    └── gait_node.py
```
