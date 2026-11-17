# LiDAR Odometry — Algorithm Selection & Rationale

## Why this module exists

The perception stack stays camera-only (see `CLAUDE.md` § Design Principles)
— this module does not change that. A lidar earns a place in the repo for
two narrower reasons:

1. **A scale-free reference for the monocular depth problem.** The project's
   current detour is solving Depth Anything V2 / Metric3D's scale error
   "kinetically" rather than by ground truth (`CLAUDE.md` § Current Target):

   - **Alternative 1** (built): `ground_plane_estimation` — compare the
     camera's RANSAC-fitted distance to the ground plane against a known
     true height. See `documentation/ground_plane_estimation.md`.
   - **Alternative 2** (this module enables it): compare the body motion
     estimated from a scale-free sensor against the motion implied by
     integrating the monocular-depth point cloud over time. Any consistent
     divergence between the two is the depth network's scale error, sampled
     continuously instead of only where a ground plane is visible.

   LiDAR range is measured directly (time-of-flight), so LiDAR odometry has
   no equivalent of monocular scale ambiguity — it is a metric ruler to
   check the monocular pipeline against, independent of `ground_plane_estimation`'s
   geometric argument.

2. **A drop-in `/odom` source for the multi-vehicle contract.** Per
   `CLAUDE.md` § Vehicle Interface Contract, everything above `/odom` +
   `/traversability_costmap` is vehicle-agnostic; only the odometry source
   differs per platform. A lidar-equipped Ackermann or tracked platform can
   plug this module in directly instead of wheel odometry.

This is **not** a replacement for `quadruped_leg_odometry` (LKO) or
`localization` (ORB-SLAM3 + EKF) on the quadruped itself — the robot has no
lidar in the current URDF or sensor suite. It is a standalone measurement
instrument that consumes whatever `PointCloud2` is on the bus: a real lidar
driver, a recorded bag, or (for experimentation) a depth-camera cloud fed
through the same topic.

---

## Requirements that shaped the choice

Given the module's actual job — a *reference* odometry for depth-scale
experiments, run against arbitrary bags rather than tightly integrated into
the quadruped's real-time control loop — the deciding constraints were:

- **No assumption about the input sensor.** The module must accept a plain
  `PointCloud2` from any lidar driver or bag without per-driver tuning
  (ring index, exact per-point timestamp semantics, extrinsic calibration
  files).
- **No hard IMU coupling.** A recorded third-party bag may not have a
  time-synced, high-rate IMU. Requiring one would make the module unusable
  as a general-purpose comparison bench.
- **Runs on the CPU.** The GPU on the Jetson Orin Nano/NX is already
  committed to Metric3D V2 inference and elevation mapping (`CLAUDE.md` §
  Environment Variables — Metric3D alone runs at ~1.2–1.5 s/frame on this
  host); a lidar odometry backend competing for the same GPU would distort
  exactly the timing behavior Alternative 2 needs to observe cleanly.
- **Few parameters, predictable failure mode.** As a measurement instrument,
  the backend's own tuning surface should not become a second experiment.

These constraints point toward a **pure-LiDAR** (not LiDAR-inertial)
odometry algorithm, which narrows the field considerably before comparing
specific implementations.

---

## Algorithms surveyed

| Algorithm | Type | Needs beyond `PointCloud2` | Maturity / integration cost |
|---|---|---|---|
| **KISS-ICP** (chosen) | Pure-LiDAR, point-to-point ICP | Nothing | pip-installable, pure Python API, ~5 tunable parameters |
| FAST-LIO2 | Tightly-coupled LiDAR-inertial (iEKF) | Hard-synced IMU (≥100 Hz typical), per-point timestamps | ROS 1/2 C++ package, per-sensor extrinsic/timing calibration |
| Point-LIO | Tightly-coupled, point-wise (no scan batching) | Same as FAST-LIO2, plus high IMU rate for point-wise updates | Newer, less battle-tested integration tooling |
| DLIO | LiDAR-inertial, continuous-time deskew | IMU + per-point timestamps | ROS 2 support exists but still IMU-coupled |
| LIO-SAM | Factor-graph LiDAR-inertial + loop closure | 9-axis IMU, `ring` field (mechanical-spinning lidar assumed) | Heaviest dependency and tuning surface (GTSAM) of the group |
| LOAM / LeGO-LOAM (historical baseline) | Feature-based (edge/planar), pure-LiDAR | Ring index, mechanical-spinning geometry assumptions | Superseded by KISS-ICP/FAST-LIO2 in accuracy and generality; included only as the lineage these descend from |

### KISS-ICP — selected

[KISS-ICP](https://github.com/PRBonn/kiss-icp) (Vizzo et al., 2023) is a
point-to-point ICP odometry pipeline built around three ideas: an adaptive
correspondence threshold derived from the model's own recent deviation
(no fixed "how far can two scans have moved" constant to tune per
environment), a voxel-hashed local map for O(1) nearest-neighbor lookups,
and a constant-velocity motion prior for the ICP initial guess. Optional
per-point motion compensation ("deskew") uses the constant-velocity
estimate itself — it does not require an IMU.

Why it fits this module specifically:

- **Sensor-agnostic by construction.** It was designed and benchmarked
  across spinning, solid-state, and non-repetitive-scan lidars without
  per-sensor changes (the paper's whole thesis is "a simple, robust
  baseline that doesn't need per-sensor tuning"). That matches "accept
  whatever `PointCloud2` shows up."
- **No IMU dependency.** The constant-velocity prior replaces what an IMU
  would otherwise supply for the ICP initial guess, so the module has
  nothing else to synchronize or calibrate. Deskewing is a pure bonus when
  a per-point time field exists (see `cloud_conversion.py`'s
  driver-agnostic time-field detection); its absence never blocks the
  pipeline, only turns deskew into a no-op.
- **CPU-only, low overhead.** Runs comfortably on the Jetson's CPU cores,
  leaving the GPU free for the depth network and elevation mapping — the
  actual pipeline this module is instrumenting.
- **Small parameter surface with sane defaults.** `max_range`/`min_range`,
  `voxel_size`, `initial_threshold`, `min_motion_th` — five knobs, all
  exposed in `lidar_odometry_node.py`. Tightly-coupled alternatives add an
  IMU noise model, extrinsic calibration, and (for LIO-SAM) loop-closure
  and factor-graph parameters on top.
- **Reference-grade accuracy for a low-dynamics reference role.** KISS-ICP
  is competitive with — and on several public benchmarks (KITTI, MulRan,
  Newer College) better than — contemporaneous LiDAR-inertial methods on
  the specific metric that matters here (absolute trajectory accuracy),
  even though it doesn't fuse inertial data. It underperforms LiDAR-inertial
  systems specifically under high angular rates / vibration — see
  Caveats.

### Why not the LiDAR-inertial alternatives (for now)

**FAST-LIO2**, **Point-LIO**, and **DLIO** are all tightly-coupled
LiDAR-inertial systems: they fuse every incoming IMU sample into an
(iterated) Kalman filter state alongside the point cloud registration,
which is what lets them stay accurate under the kind of high angular
velocity and vibration a *trotting* legged robot produces — exactly the
regime `quadruped_leg_odometry`'s docstring calls out as LKO's weak point
(`README.md`: "Drift on slippery/uneven terrain"). If a lidar is ever
body-mounted on the quadruped for production odometry (not just as a
scale-reference bench), this is the right upgrade path, in roughly this
order:

- **FAST-LIO2** first — the most mature, widely deployed of the three, with
  the most existing ROS 2 integration precedent.
- **Point-LIO** if FAST-LIO2's scan-batched updates prove too coarse for the
  motion profile — it updates per-point rather than per-scan, trading
  implementation complexity for robustness under more aggressive dynamics.
- **DLIO** as a closely comparable alternative to FAST-LIO2, notable mainly
  for continuous-time deskewing baked into the front end.

All three require the same two things this module was built to avoid
depending on: a hard-synchronized, adequately-rated IMU, and per-point
timestamps in the cloud. Bags recorded with an arbitrary lidar (the actual
near-term use case — Alternative 2 above) frequently have neither in a form
the module could rely on. Adding that dependency now would turn a
measurement instrument into another integration project.

**LIO-SAM** was set aside for the same IMU/timestamp reasons, plus a
heavier dependency (GTSAM, factor-graph tuning) and a `ring`-field
assumption that presumes a mechanical spinning lidar — the least
sensor-agnostic option surveyed.

**LOAM/LeGO-LOAM** are included only for lineage: they pioneered
feature-based (edge/planar) LiDAR odometry, but KISS-ICP's dense
point-to-point approach has since matched or exceeded their accuracy while
needing less per-sensor tuning (no explicit edge/planar feature
extraction tied to a specific scan pattern), and FAST-LIO2 is the modern
inheritor of the tightly-coupled line these older methods only
loosely approximated. Neither was seriously considered as a
current candidate.

### Decision summary

| Requirement | KISS-ICP | FAST-LIO2 / Point-LIO / DLIO | LIO-SAM |
|---|:---:|:---:|:---:|
| Works on an arbitrary bag with no IMU | ✅ | ❌ | ❌ |
| No per-point timestamp requirement | ✅ (optional) | ❌ (required for deskew) | ❌ |
| CPU-only | ✅ | ✅ (but IMU fusion overhead) | ✅ |
| Robust under legged-robot vibration/rotation rates | ⚠️ moderate | ✅ strong | ✅ strong |
| Parameter/integration surface | Small | Larger (IMU noise, extrinsics) | Largest (+ loop closure) |

KISS-ICP wins on every axis this module's current job actually needs
(sensor-agnostic, IMU-free, CPU-only, small surface) and only loses on the
one axis (vibration robustness) that matters for production on-robot
odometry, which is explicitly not this module's current job. The trade-off
is revisited the day a lidar is mounted on the quadruped itself — see
Caveats.

---

## Pipeline

```
PointCloud2 (any frame, any lidar driver / bag)
        │
        ▼
   extract_xyz_and_times()       (cloud_conversion.py: drop non-finite
        │                         points; detect + normalize an optional
        │                         per-point time field, driver-agnostic)
        ▼
   KissICP.register_frame()      (adaptive-threshold point-to-point ICP
        │                         against a voxel-hashed local map, with
        │                         a constant-velocity initial guess)
        ▼
   conjugate with the static
   base_link ← <cloud frame>     (T_base = T_bl · T_lidar · T_bl⁻¹;
   extrinsic (looked up once      "odom" = base_link pose at scan 0)
   from TF, identity fallback)
        │
        ▼
   /odom_lidar  (nav_msgs/Odometry: odom → base_link, twist in base_link,
                 drift-model covariance)
```

### Frame handling

KISS-ICP estimates the *sensor's* motion in the frame of its own first
scan. To make the output obey the vehicle interface contract (`odom` →
`base_link`, per `CLAUDE.md` § Vehicle Interface Contract), the node:

1. Looks up the static `base_link ← <cloud frame>` transform once via TF
   (`_resolve_extrinsic`), waiting up to `wait_for_extrinsic_sec` (default
   10 s) for `robot_state_publisher` or a `static_transform_publisher` to
   provide it.
2. If the extrinsic never appears — the realistic case for a standalone bag
   replay with no matching URDF/TF — it falls back to identity after the
   grace period and logs a warning; the odometry is then reported directly
   in the lidar's own frame (still internally consistent, just not
   `base_link`-referenced).
3. Every subsequent pose/delta from KISS-ICP is conjugated by this fixed
   extrinsic: `T_base = T_bl · T_lidar · T_bl⁻¹`. This is a similarity
   transform, so it correctly carries both the rotation and the lever-arm
   effect of an off-center lidar mount into the base-frame twist.

Twist (linear/angular velocity) is computed as `delta_base / dt` rather
than differentiating the accumulated pose, to keep per-sample noise from
compounding through the integration.

### Uncertainty

Following the same explicit-uncertainty convention as
`quadruped_leg_odometry`'s `/odom_lko`:

- **Twist covariance** is a fixed per-sample estimate
  (`twist_linear_stddev`, `twist_angular_stddev`) — KISS-ICP does not
  expose a native per-scan confidence, so this is a static budget rather
  than something the node infers from ICP inlier quality.
- **Pose covariance** grows with integrated distance and turned angle
  (`drift_per_metre`, `drift_per_radian`), the same drift-model shape used
  by LKO — this is scan-to-map ICP odometry, not a loop-closed SLAM
  estimate, so open-loop drift is inevitable and must be represented, not
  hidden.

---

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `input_topic` | `/lidar/points` | Input `PointCloud2` |
| `odom_topic` | `/odom_lidar` | Output `nav_msgs/Odometry` |
| `base_frame` | `base_link` | Body frame for the published odometry |
| `publish_tf` | `false` | Broadcast `odom→base_link` — debug only; the EKF/ground-truth publisher normally owns this edge |
| `max_range` / `min_range` | `30.0` / `0.3` | KISS-ICP range gate (m) |
| `voxel_size` | `0.0` (auto = `max_range/100`) | Local map voxel size (m) |
| `max_points_per_voxel` | `20` | Cap on points retained per voxel in the local map |
| `initial_threshold` | `2.0` | Adaptive-threshold estimator's starting correspondence distance |
| `min_motion_th` | `0.1` | Minimum motion below which the adaptive threshold stops shrinking |
| `deskew` | `false` | Motion-compensate scans; silently degrades to off if the cloud has no per-point time field |
| `wait_for_extrinsic_sec` | `10.0` | Grace period for the `base_link←cloud` TF extrinsic before falling back to identity |
| `twist_linear_stddev` / `twist_angular_stddev` | `0.05` / `0.02` | 1-sigma twist noise (m/s, rad/s) |
| `drift_per_metre` / `drift_per_radian` | `0.01` / `0.01` | 1-sigma pose drift per metre traveled / radian turned |

---

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/lidar/points` (remappable via `input_topic`) | `sensor_msgs/PointCloud2` | in | Any lidar driver, bag, or (for experimentation) a depth-camera cloud |
| `/odom_lidar` | `nav_msgs/Odometry` | out | `odom → base_link`, twist in `base_link`, drift-model covariance |
| TF `base_link ← <cloud frame>` | static | in | Extrinsic; identity fallback after `wait_for_extrinsic_sec` if absent |

---

## Running it

```bash
# Idles until /lidar/points appears — part of the root compose:
docker compose up -d --build lidar_odometry

# Against a recorded bag:
LIDAR_POINTCLOUD_TOPIC=/ouster/points LIDAR_DESKEW=true \
  docker compose up -d --build lidar_odometry
ros2 bag play <bag>   # separately

# Compare against the depth-scale probe / leg odometry (Alternative 2):
ros2 topic echo /odom_lidar
ros2 topic echo /odom_lko
```

---

## Caveats

- **Not fused with IMU.** By design (see rationale above). Under the
  quadruped's actual trot dynamics — should a lidar ever be body-mounted —
  KISS-ICP's pure-geometric registration is more exposed to motion blur and
  aggressive angular rates than a tightly-coupled filter would be. This
  module targets bag-driven reference use, not on-robot production
  odometry; re-evaluate FAST-LIO2 (see rationale above) if that changes.
- **Open-loop drift, no loop closure.** Unlike `localization`'s
  ORB-SLAM3 (which corrects drift via `map→odom`), KISS-ICP's local map
  only ever grows forward — the published pose covariance models expected
  drift but nothing corrects it. Do not treat `/odom_lidar` as a
  localization signal.
- **Identity-extrinsic fallback changes the output frame silently
  (logged, not raised).** Any consumer expecting a true `base_link`-relative
  signal must confirm the extrinsic actually resolved (check the log for
  "TF ... never appeared") when running against a bag without a matching
  robot description.
- **Deskew quality depends entirely on whether the source driver populates
  a recognized per-point time field** (`t`, `time`, `timestamp`,
  `time_offset`, `point_time_offset`, `stamps` — see
  `cloud_conversion.py`). An unrecognized field name silently disables
  deskew rather than failing, which is intentional (never block on a
  missing optional field) but worth checking in the startup log if
  motion-compensation quality looks off.
- **No lidar exists in the current Gazebo model.** The quadruped's URDF
  (`src/quadruped_gazebo/urdf/quadruped.urdf.xacro`) has only a depth
  camera, RGB camera, and IMU. This module currently has no in-sim
  validation path; it is built and tested against synthetic point clouds
  (`test/test_node_smoke.py`) and is intended for bag-driven use until/unless
  a `gpu_lidar` sensor is added to the sim.

---

## Source layout

| File | Role |
|---|---|
| `lidar_odometry/cloud_conversion.py` | `PointCloud2` → numpy; driver-agnostic per-point time-field detection and normalization |
| `lidar_odometry/lidar_odometry_node.py` | ROS node: KISS-ICP pipeline, TF extrinsic resolution, drift-model covariance, publishing |
| `launch/lidar_odometry.launch.py` | Launch file with all node parameters exposed as launch arguments |
| `test/test_cloud_conversion.py` | Host-runnable: time normalization (relative seconds, absolute nanoseconds, degenerate inputs), row-aligned filtering |
| `test/test_node_smoke.py` | ROS-environment: synthetic translating scene → `/odom_lidar` within 10 s |
| `README.md` | Module-local quick reference (parameters, run commands, algorithm comparison table) |
