# Nav2 MPPI Controller — Detailed Reference

## What is MPPI?

**Model Predictive Path Integral (MPPI)** is a sampling-based optimal control algorithm. At each control step it:

1. Draws `batch_size` random perturbations around the current control sequence.
2. Rolls each sequence forward `time_steps` into the future using the robot's motion model.
3. Scores every trajectory with a set of weighted cost functions (critics).
4. Combines all trajectories into a single optimal command using an exponentially-weighted average — trajectories with lower cost get more weight.
5. Shifts the horizon forward by one step and repeats.

Unlike pure-pursuit or DWA controllers, MPPI reasons over a full time horizon and handles non-convex cost landscapes naturally.

---

## Integration in Visual Paws

```
/occupancy_grid  ──▶  costmap  ──▶  CostCritic
/odom            ──▶  MPPI state estimation
/tf              ──▶  frame transforms
/plan            ──▶  path critics (PathAlign, PathFollow, PathAngle)

MPPI  ──▶  /cmd_vel_smoothed  ──▶  velocity_smoother  ──▶  /cmd_vel
                                                              │
                                                              ▼
                                                    gait_controller
                                                    (IK + joint cmds)
```

Goals enter via the `navigate_to_pose` action (RViz2 "2D Nav Goal"). The BT Navigator runs the global planner (NavFn) first, then hands the path to the MPPI controller server.

---

## Sampling Parameters

| Parameter | Value | Effect |
|---|---|---|
| `batch_size` | 2 000 | Number of parallel trajectory rollouts per control step. Higher = better coverage of the state space, higher CPU cost. |
| `time_steps` | 56 | Steps per rollout. Horizon = 56 × 0.05 s = **2.8 s**. |
| `model_dt` | 0.05 s | Integration timestep within each rollout. |
| `iteration_count` | 1 | How many MPPI update passes per control cycle. Values > 1 refine the solution but add latency. |
| `temperature` | 0.3 | Sharpness of the exponential weighting. Lower → winner-takes-all; higher → smoother blend. |
| `gamma` | 0.015 | Discount factor applied to control costs along the horizon. |
| `reset_period` | 1.0 s | If no command is received for this long, the warm-start sequence is cleared. |

---

## Motion Model

Current setting: **`DiffDrive`**

The DiffDrive model constrains lateral velocity to zero (`vy = 0`). The controller samples only `vx` and `wz`.

### Switching to Omnidirectional

To use full 3-DOF planar motion, change two lines in `nav2_mppi_params.yaml`:

```yaml
motion_model: "Omni"
vy_max: 0.5       # currently 0.0
```

This also requires the gait controller to accept non-zero `cmd_vel.linear.y` commands.

### Noise Parameters (standard deviations for sampling)

| Parameter | Value | Applies to |
|---|---|---|
| `vx_std` | 0.2 m/s | Forward/reverse velocity perturbation |
| `vy_std` | 0.2 m/s | Lateral velocity perturbation (active only in Omni mode) |
| `wz_std` | 0.4 rad/s | Yaw rate perturbation |

### Velocity Limits

| Limit | Value |
|---|---|
| `vx_max` | 0.5 m/s |
| `vx_min` | −0.35 m/s |
| `vy_max` | 0.0 m/s (DiffDrive) |
| `wz_max` | 1.9 rad/s |
| `ax_max` / `ax_min` | ±3.0 m/s² |
| `ay_max` | 3.0 m/s² |
| `az_max` | 3.5 rad/s² |

---

## Critics

Critics are cost functions evaluated over every sampled trajectory. Each critic returns a scalar cost; the weighted sum drives the MPPI update. All critics use `cost_power: 1` (linear scaling).

### ConstraintCritic — weight 4.0

Penalises any trajectory that violates the declared velocity or acceleration limits. Acts as a hard guardrail; trajectories that exceed limits accumulate cost quickly.

### GoalCritic — weight 5.0

Penalises distance from the goal position. Only active within `threshold_to_consider: 1.4 m` of the goal so it does not compete with path-following critics during long-range travel.

### GoalAngleCritic — weight 3.0

Penalises heading error at the goal. Active within `threshold_to_consider: 0.5 m`. Ensures the robot arrives facing the correct direction.

### PathAlignCritic — weight 14.0 *(highest)*

Penalises lateral deviation from the planned path. The high weight makes this the dominant shaping force during straight-line tracking.

Key sub-parameters:

| Sub-parameter | Value | Meaning |
|---|---|---|
| `trajectory_point_step` | 4 | Sample every 4th trajectory point for efficiency |
| `offset_from_furthest` | 20 | Look 20 points behind the furthest reached path point |
| `max_path_occupancy_ratio` | 0.05 | If > 5 % of the path is in occupied cells, skip alignment (obstacle avoidance takes priority) |
| `use_path_orientations` | false | Compare positions only, not headings |

### PathFollowCritic — weight 5.0

Drives progress along the path by rewarding the trajectory that reaches the furthest path point. Prevents the robot from stalling.

| Sub-parameter | Value | Meaning |
|---|---|---|
| `offset_from_furthest` | 5 | Look 5 points behind furthest reached point |
| `threshold_to_consider` | 1.4 m | Deactivates near the goal (GoalCritic takes over) |

### PathAngleCritic — weight 2.0

Penalises heading misalignment relative to the path direction. `forward_preference: true` adds an extra cost for trajectories that approach the path while facing backward.

| Sub-parameter | Value |
|---|---|
| `offset_from_furthest` | 4 |
| `max_angle_to_furthest` | 1.0 rad |
| `threshold_to_consider` | 0.5 m |

### CostCritic — weight 3.81

Obstacle avoidance using the 2D costmap. Each trajectory point is looked up in the costmap and penalised proportionally.

| Sub-parameter | Value | Meaning |
|---|---|---|
| `consider_footprint` | true | Use full robot polygon instead of point check |
| `collision_cost` | 1 000 000 | Effectively infinite cost — any trajectory through an occupied cell is discarded |
| `critical_cost` | 300.0 | Threshold above which a cell is treated as lethal |
| `near_goal_distance` | 1.0 m | Within this radius the critic is relaxed to avoid getting stuck near the goal |
| `trajectory_point_step` | 2 | Check every 2nd point for efficiency |

### PreferForwardCritic — weight 5.0

Adds a cost penalty for backward motion. Keeps the robot facing its direction of travel for predictable behaviour. Active within `threshold_to_consider: 0.5 m` of the goal.

---

## Costmap

Both costmaps read the live `/occupancy_grid` topic published by `elevation_mapping_cupy` (no static map file required).

```
/occupancy_grid  ──▶  static_layer  ──▶  inflation_layer  ──▶  costmap
```

### Inflation Layer

Obstacle cost decays exponentially with distance from the obstacle boundary:

```
cost = 253 × e^(−cost_scaling_factor × (distance − inscribed_radius))
```

| Parameter | Value |
|---|---|
| `inflation_radius` | 0.55 m |
| `cost_scaling_factor` | 3.0 |

The robot footprint polygon (0.40 m × 0.30 m) defines the inscribed radius used for footprint checking in CostCritic.

### Local vs Global Costmap

| | Local | Global |
|---|---|---|
| Frame | `odom` | `map` |
| Window | 4 m × 4 m rolling | full `/occupancy_grid` extent |
| Update rate | 5 Hz | 1 Hz |
| Used by | MPPI controller | NavFn planner |

---

## Velocity Pipeline

After the MPPI controller outputs a command, it passes through two more nodes before reaching the gait controller:

```
MPPI  →  /cmd_vel_smoothed  →  velocity_smoother  →  collision_monitor  →  /cmd_vel
```

### velocity_smoother

Applies acceleration limits in open-loop mode (no odometry feedback required):

| Limit | Value |
|---|---|
| Max velocity | [0.5, 0.0, 2.5] m/s, m/s, rad/s |
| Min velocity | [−0.5, 0.0, −2.5] |
| Max accel | [2.5, 0.0, 3.2] |
| Max decel | [−2.5, 0.0, −3.2] |
| Smoothing frequency | 20 Hz |

### collision_monitor

Currently a pass-through (`observation_sources: ""`). To activate, add polygon or point cloud observation sources and define stop/slowdown zones.

---

## Global Planner — NavFn

| Parameter | Value |
|---|---|
| Plugin | `nav2_navfn_planner::NavfnPlanner` |
| Algorithm | Dijkstra (`use_astar: false`) |
| Goal tolerance | 0.5 m |
| Allow unknown space | true |
| Expected frequency | 20 Hz (replanning if path becomes invalid) |

---

## Recovery Behaviours

Managed by `behavior_server`; triggered automatically by the BT Navigator when the robot is stuck.

| Behaviour | Description |
|---|---|
| `spin` | Rotate in place to escape local minima |
| `backup` | Drive backward a short distance |
| `drive_on_heading` | Drive forward on current heading |
| `wait` | Pause and wait for the environment to change |

Recovery limits: max rotational velocity 1.0 rad/s, min 0.4 rad/s, acceleration limit 3.2 rad/s².

---

## Goal Tolerance & Progress Checking

| Parameter | Value |
|---|---|
| XY goal tolerance | 0.25 m |
| Yaw goal tolerance | 0.25 rad |
| Required movement radius | 0.5 m |
| Movement time allowance | 10.0 s — if the robot does not move 0.5 m within 10 s, a recovery is triggered |

---

## Localisation

A `static_transform_publisher` broadcasts a fixed identity transform from `map` to `odom`. This makes the two frames coincide, so NavFn plans in the same frame the MPPI controller tracks. Suitable for simulation; replace with AMCL or another localisation node for real-world deployment.

---

## Trajectory Visualisation

With `visualize: true`, the MPPI controller publishes sampled trajectory markers to RViz. Density is controlled by:

- `trajectory_step: 5` — publish every 5th rollout
- `time_step: 3` — show every 3rd time step within each rollout

This is useful for tuning critic weights: a well-tuned controller shows a tight bundle of trajectories tracking the planned path, with the optimal trajectory clearly separated from obstacle-adjacent ones.
