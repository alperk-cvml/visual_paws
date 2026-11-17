# lidar_odometry

LiDAR odometry from any `sensor_msgs/PointCloud2`, using **KISS-ICP** as the
backend. Publishes `/odom_lidar` (`nav_msgs/Odometry`) per the vehicle
interface contract.

```
/lidar/points ──► KISS-ICP (adaptive-threshold ICP, constant-velocity
                  prior, voxel hash map) ──► /odom_lidar
```

## Why this module exists in a camera-only stack

The perception stack stays camera-only. A lidar earns its place here in two
supporting roles:

1. **Metric reference odometry** for the monocular depth scale problem
   (Alternative 2 of the current detour): lidar odometry has no scale
   ambiguity, so comparing `/odom_lidar` against motion inferred from the
   monodepth point cloud isolates the depth network's scale error.
2. **Drop-in odometry source** for other platforms of the multi-vehicle
   contract (Ackermann / tracked vehicles that carry a lidar anyway).

## Algorithm choice

See `documentation/lidar_odometry.md` for the full rationale (requirements,
per-algorithm trade-offs, and the decision table). Summary:

| Algorithm | Type | Needs | Verdict for this project |
|---|---|---|---|
| **KISS-ICP** (chosen) | pure LiDAR ICP | nothing beyond `PointCloud2` | Sensor-agnostic, ~5 parameters, CPU-only, pip-installable, no IMU/ring/per-point-time requirements. Works on bags from any lidar, which is exactly the benchmark-reference use case. |
| FAST-LIO2 | tightly-coupled LiDAR-inertial | hard-synced high-rate IMU, per-point timestamps, C++ build | Better on a trotting body (severe inter-scan shake), but the sync requirement makes it wrong for replayed third-party bags. Upgrade path if a lidar is body-mounted for production. |
| Point-LIO | tightly-coupled, point-wise updates | same as FAST-LIO2, even higher IMU rates | Best-in-class under aggressive motion; same integration burden. |
| DLIO | LiDAR-inertial, continuous-time deskew | IMU + per-point timestamps | Comparable to FAST-LIO2; official ROS 2 support, still IMU-coupled. |
| LIO-SAM | factor-graph LiDAR-inertial | 9-axis IMU, `ring` field, heavier tuning | Mature but the most demanding input contract of the list. |

Rule of thumb: start pure-lidar (KISS-ICP) while the module is a measurement
instrument; go LiDAR-inertial (FAST-LIO2 / Point-LIO) only when the odometry
must survive locomotion-grade dynamics on the robot itself. The node keeps the
backend behind a single class so a second backend can be added the way
`depth_comparison` registers depth backends.

## Interface

| Topic | Type | Direction |
|---|---|---|
| `/lidar/points` | `sensor_msgs/PointCloud2` | in (remap via `input_topic`) |
| `/odom_lidar` | `nav_msgs/Odometry` | out (`odom` → `base_link`, twist in `base_link`) |
| TF `base_link ← <cloud frame>` | static | in (extrinsic; identity fallback after `wait_for_extrinsic_sec`) |

- **No TF by default.** `odom→base_link` belongs to the robot_localization
  EKF (or the sim ground-truth publisher). `publish_tf:=true` only for
  standalone debugging.
- KISS-ICP estimates the sensor's motion; the node conjugates it with the
  static `base_link←lidar_link` extrinsic so the published odometry is
  `base_link` motion, with `odom` anchored at the first scan's base pose.
- Pose covariance grows with traveled distance / turned angle
  (`drift_per_metre`, `drift_per_radian`) — this is odometry, not
  localization.
- A backward jump in input stamps > 1 s (bag restart) resets the pipeline.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `input_topic` | `/lidar/points` | Input cloud |
| `odom_topic` | `/odom_lidar` | Output odometry |
| `base_frame` | `base_link` | Body frame for the output |
| `max_range` / `min_range` | `30.0` / `0.3` | Range gate (m) |
| `voxel_size` | `0.0` (auto = `max_range/100`) | Local map voxel size (m) |
| `deskew` | `false` | Motion compensation; needs a per-point time field (`t`, `time`, `timestamp`, …) |
| `publish_tf` | `false` | Broadcast `odom→base_link` (debug only) |
| `wait_for_extrinsic_sec` | `10.0` | Grace period for the TF extrinsic before identity fallback |

## Run

```bash
# Part of the root compose (idles until /lidar/points appears):
docker compose up -d --build lidar_odometry

# Against a bag:
LIDAR_POINTCLOUD_TOPIC=/ouster/points LIDAR_DESKEW=true \
  docker compose up -d --build lidar_odometry
ros2 bag play <bag>   # in another container/terminal
```

## Tests

```bash
colcon build --symlink-install --packages-select lidar_odometry
pytest src/lidar_odometry/test -q
```

`test_cloud_conversion.py` covers the PointCloud2 → numpy path (time-field
normalization is pure numpy and runs anywhere); `test_node_smoke.py` drives
the node with a synthetic scene translating at 0.5 m/s and asserts the
recovered odometry within 10 s. ROS-dependent tests skip outside a ROS
environment.
