# quadruped_gait_controller

Spot Micro gait controller. Turns `/cmd_vel` into 12 joint position commands
with a Bézier-swing gait and whole-body inverse kinematics.

```
/cmd_vel (Twist) ──► clamp · accel ramp · watchdog
                          │
                          ▼
                  BezierGait (gait.py)        footholds in the nominal body frame N
                          │
                          ▼
                  body_ik (quadruped_kinematics)   body pose + 4 × leg IK
                          │
                          ▼
        /quadruped_joint_controller/commands (Float64MultiArray, 12)
```

Joint order is `quadruped_kinematics.JOINT_ORDER`, the same list as
`quadruped_gazebo/config/ros2_control.yaml`:
`FL hip_roll, hip_pitch, knee, FR …, RL …, RR …`.

## Gaits

| `gait_type` | Cycle | Duty | Swing | Body shift | Use |
|---|---|---|---|---|---|
| `trot` (default) | 0.4 s | 0.70 | 0.12 s, 20 mm lift | off | Everyday locomotion: 0.2–0.3 m/s, turns and strafes cleanly |
| `walk` | 1.0 s | 0.75 | 0.25 s, 30 mm lift | 0.8 into the support triangle | Slow, statically stable crawl (Spot Micro's 8-phase walk, made continuous) |

- **Stance**: the foot sweeps linearly from +Δ/2 to −Δ/2, where Δ is the
  body's ground displacement over one stance period at the commanded
  velocity, evaluated at that foot (`v + ω × r`), so yaw becomes tangential
  strides. All legs share one scale factor when any Δ exceeds
  `max_step_length`, which keeps a turn a turn.
- **Swing**: 11th-order Bézier (Hyun et al. 2014, the MIT Cheetah /
  spot_mini_mini curve) from −Δ/2 back to +Δ/2, tangent to the ground at both
  ends so touchdown is soft. Stance and swing meet exactly, so foot position
  is continuous wherever the command changes.
- **Stopping**: on a zero command the current cycle completes, so every foot
  is back at its neutral foothold before the robot holds a static stand.
- **Body shift** (`walk`): the body is moved toward the centroid of the feet
  in stance, low-passed. With one foot in the air that puts the centre of
  mass inside the support triangle instead of on its edge.
- **Trot** pairs FL+RR and FR+RL. Duty 0.7 gives short swings with four-foot
  support in between. A duty-0.5 flight trot tips over on its diagonal under
  position control (see the tuning notes in `gait.py`).

## Parameters (`config/gait_params.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `gait_type` | `trot` | `trot` or `walk`; selects the tuned defaults below |
| `period`, `duty`, `swing_clearance`, `body_shift_gain`, `body_shift_tau` | 0 / −1 = gait default | Override the gait's tuned value with any positive number |
| `max_step_length` | 0.10 m | Foot travel per cycle; caps speed |
| `stance_depth` | 0 | Push below nominal ground in stance (off) |
| `body_height` | 0.16 m | base_link above ground (matches the URDF `stand_height`) |
| `body_x`, `body_y`, `body_roll`, `body_pitch`, `body_yaw` | 0 | Static body pose over the feet |
| `foot_x_offset`, `foot_y_offset` | 0 | Move the neutral footholds fore/aft or wider |
| `max_vx`, `max_vy`, `max_wz` | 0.4, 0.25, 1.2 | Command clamps |
| `linear_accel`, `angular_accel` | 0.6, 2.5 | Ramp limits |
| `cmd_vel_timeout` | 0.5 s | Stop if `/cmd_vel` goes stale |
| `crouch_height`, `stand_up_duration` | 0.09 m, 1.0 s | Start-up ramp from crouch to stand |
| `control_rate` | 100 Hz | Publish rate |

Link lengths are not parameters. They come from `quadruped_kinematics`
(`SPOT_MICRO`) and are checked against the URDF by that package's tests.

## Run

```bash
# via the root docker compose (container quadruped_gait_controller)
GAIT_TYPE=walk docker compose up -d quadruped_gait_controller

# inside a container
ros2 launch quadruped_gait_controller gait.launch.py gait_type:=trot use_sim_time:=true
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

## Tests

```bash
# pure-python (no ROS needed)
PYTHONPATH=src/quadruped_kinematics:src/quadruped_gait_controller \
  python3 -m pytest src/quadruped_gait_controller/test
```

`test_gait.py` checks gait timing, stride scaling, continuity, Bézier shape
and that every foot target stays inside the IK workspace at the command
clamps. `test_node_smoke.py` (needs `rclpy`) drives the node through
stand-up → walking → stand.

## Validation in Gazebo (headless, flat ground, ground-truth pose)

| Test | Result |
|---|---|
| Trot forward 0.2 m/s, 7 s | 1.33 m, heading drift +6°, roll/pitch < 2°, body z within 1 mm |
| Trot forward 0.3 m/s | 1.53 m in 6.2 s, heading drift −7° |
| Trot turn 0.8 rad/s | 0.805 rad/s measured, roll/pitch < 1.5° |
| Trot strafe 0.15 m/s | 0.98 m in 6.5 s, 2° body motion |
| Walk forward 0.2 m/s (capped to ~0.17 by step length) | 1.53 m, heading −2° |
| Leg odometry vs ground truth | turn 0.805 vs 0.805 rad/s; strafe 0.148 vs 0.15 m/s; forward trot 0.155 vs 0.184 m/s (swing feet skid slightly at touchdown) |

Reproduce with `gazebo.launch.py headless:=true` and `localization_mode:=ground_truth`.
