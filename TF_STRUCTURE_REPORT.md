# Visual Paws TF Structure Report

## Scope and method

This report describes the TF structure implemented in the repository as of 2026-07-27. It is based on the active launch files, URDF, node source, and configuration for Gazebo, localization, elevation mapping, and Nav2. References in vendored demonstrations and tests are excluded unless the active Visual Paws wrapper inherits their settings.

The repository has two relevant operating modes:

- **Ground-truth simulation:** Gazebo pose data supplies `odom -> base_link`, with a temporary identity `map -> odom`.
- **SLAM localization:** ORB-SLAM3 and Gazebo's temporary continuous odometry source feed separate global and local EKFs.
- **Offline video:** ORB-SLAM3 directly supplies `map -> base_link`; static camera extrinsics complete the point-cloud transform without Gazebo or an EKF.

The intended navigation convention is the standard hierarchy:

```text
map -> odom -> base_link -> robot links and sensors
```

The implementation now constructs that hierarchy in both supported localization modes. The remaining limitations are documented below.

## Frame inventory

| Frame | Meaning | Defined or used by |
|---|---|---|
| `quadruped_world` | Gazebo world frame for the model pose | Gazebo pose publisher and `gazebo.launch.py` |
| `quadruped/base_link` | Gazebo-scoped model base frame | Gazebo model pose bridge |
| `map` | Globally stable navigation/SLAM frame | Nav2, ORB-SLAM3 odometry, localization configuration |
| `odom` | Locally continuous odometry and elevation-map frame | Nav2 local costmap, elevation mapping, EKF |
| `base_link` | Robot body and root of the URDF link tree | URDF, Nav2, mapping, localization |
| `imu_link` | IMU sensor frame at the body origin | Fixed URDF joint; IMU message frame |
| `camera_link` | Body-fixed camera mount frame | Fixed URDF joint; depth camera frame |
| `camera_link_optical` | ROS optical frame: Z forward, X right, Y down | Fixed URDF joint; RGB camera/video frame |
| `{FL,FR,RL,RR}_hip_link` | Per-leg hip links | URDF revolute joints |
| `{FL,FR,RL,RR}_thigh_link` | Per-leg thigh links | URDF revolute joints |
| `{FL,FR,RL,RR}_calf_link` | Per-leg calf links | URDF revolute joints |
| `{FL,FR,RL,RR}_foot_link` | Per-leg foot links | URDF fixed joints |

## Robot-local TF tree

`robot_state_publisher` receives the Xacro-generated `robot_description`. Fixed joints are published on `/tf_static`; the 12 actuated leg joints are published on `/tf` from `/joint_states`, which is supplied by the ROS 2 control joint-state broadcaster.

```text
base_link
├── imu_link                                  fixed: xyz 0 0 0
├── camera_link                              fixed: x=0.125, y=0, z=0.01375
│   └── camera_link_optical                  fixed: rpy=(-pi/2, 0, -pi/2)
├── FL_hip_link -> FL_thigh_link -> FL_calf_link -> FL_foot_link
├── FR_hip_link -> FR_thigh_link -> FR_calf_link -> FR_foot_link
├── RL_hip_link -> RL_thigh_link -> RL_calf_link -> RL_foot_link
└── RR_hip_link -> RR_thigh_link -> RR_calf_link -> RR_foot_link
```

Each leg begins at `x=+/-0.10 m`, `y=+/-0.065 m` on `base_link`. Hip roll, hip pitch, and knee are dynamic revolute joints; the calf-to-foot transform is fixed.

The camera optical rotation is consistent with the ROS optical convention. The RGB sensor explicitly stamps data as `camera_link_optical`. The simulated depth sensor, however, stamps data as `camera_link`, so downstream consumers must use TF to account for the camera mount but should not assume the depth points use optical axes without verifying the Gazebo bridge's point-cloud convention.

## Ground-truth simulation mode

Gazebo's model pose is bridged to `/gazebo/model_pose`, not directly to `/tf`. `gazebo_ground_truth_odometry.py` selects the model's base pose and publishes `/odom_ground_truth`. In `ground_truth` mode it also owns the dynamic `odom -> base_link` transform. A static publisher owns the identity `map -> odom`.

```text
map -> odom -> base_link -> URDF links
```

`robot_state_publisher` remains the only owner of transforms below `base_link`.

## SLAM localization mode

### ORB-SLAM3 output

`orbslam3_rgbd_node` publishes `/odom_slam` with:

```text
header.frame_id = map
child_frame_id  = base_link
```

ORB-SLAM3's optical-frame pose is converted to ROS body axes. The node then looks up `camera_link <- base_link` at the image timestamp and applies the URDF camera lever arm before labelling the result as the `base_link` pose. If the extrinsic is unavailable, the measurement is skipped instead of publishing a mislabelled pose.

In `slam` mode direct TF publication is disabled, leaving TF ownership to the EKFs.

### Dual EKF output

The local filter in `ekf_local.yaml` consumes `/odom_ground_truth` and `/imu/data`, publishes `/odom`, and solely owns `odom -> base_link`. Ground truth is a temporary simulation input until leg-kinematics odometry exists.

The global filter in `ekf_global.yaml` consumes `/odom` and `/odom_slam`, publishes `/odometry/global`, and solely owns `map -> odom`. ORB-SLAM never broadcasts TF directly.

Both Gazebo and localization launches accept the same `localization_mode` choice:

- `ground_truth`: Gazebo owns both navigation transforms; localization EKFs are disabled.
- `slam`: Gazebo publishes odometry data without TF; the two EKFs own the navigation transforms.
- `offline_video`: both EKFs are disabled and ORB-SLAM owns `map -> base_link`.

## Offline-video localization mode

The standalone profile in `docker-compose-camera-pipeline.yaml` does not include Gazebo. The localization launch publishes the camera extrinsics:

```text
map --(ORB-SLAM3)--> base_link
    --(static)-----> camera_link
    --(static)-----> camera_link_optical
```

RGB, inferred metric depth, camera info, and the resulting point cloud are stamped in `camera_link_optical`. The CuPy mapper is overridden to use `map`, so it can directly resolve `map <- camera_link_optical`.

The profile uses wall-clock time (`USE_SIM_TIME=false`) throughout and RGB-D ORB-SLAM (`MODE=RGBD`) to retain metric translation scale. Neither `/clock` nor `/odom_ground_truth` is required.

Playback is delayed by `VIDEO_START_DELAY` (20 seconds in the supplied profile) so ORB vocabulary and depth-model initialization finish before the first non-looping video frame is published.

Run it with:

```bash
docker compose --env-file offline-video.env \
  -f docker-compose-camera-pipeline.yaml up --build
```

## TF consumers

### Elevation mapping

The active CuPy wrapper defaults to:

```text
map_frame       = odom
base_frame      = base_link
corrected frame = odom
```

The offline profile overrides the map and corrected-map frames to `map`.

For each `PointCloud2`, the mapper reads `header.frame_id` and looks up:

```text
odom <- point_cloud_frame
```

It separately looks up `odom <- base_link` at the point-cloud timestamp to center and orient the rolling map. If either path is absent, disconnected, or temporally unavailable, that update is skipped. Published grid maps are stamped in `odom`.

The wrapper exposes `map_frame_id` and `base_frame_id`. The obsolete Compose `sensor_frame_id` argument was removed; the mapper correctly relies on the incoming point cloud's header.

### Nav2

Nav2's relevant frame expectations are:

| Consumer | Global frame | Robot frame |
|---|---|---|
| BT navigator | `map` | `base_link` |
| Global costmap | `map` | `base_link` |
| Local costmap | `odom` | `base_link` |
| Behavior server | `odom` | `base_link` |
| Docking server | `odom` | `base_link` |

Thus Nav2 requires continuous paths for both `map -> base_link` and `odom -> base_link`, normally supplied by `map -> odom -> base_link`.

The Nav2 launch substitutes `map_frame`, `odom_frame`, `base_frame`, and occupancy-topic arguments into marked YAML values before starting Nav2, so Compose overrides affect the active configuration.

The collision monitor is disabled by the launch. Its dormant configuration sets both `base_frame_id` and `odom_frame_id` to `camera_link`, an intentional bypass rather than a valid navigation-frame model; it should not be reused if collision monitoring is enabled.

### Image and sensor producers

- Gazebo IMU messages use `imu_link`.
- Gazebo RGB messages use `camera_link_optical`.
- Gazebo depth messages/points are configured with `camera_link`.
- `video_camera_sim` defaults images and camera info to `camera_link_optical` but does not publish that transform; another component must provide the robot description/static camera transform.
- `monocular_depth` preserves the input image header, so generated depth stays in the source camera frame.

## Intended ownership model

A clean split of TF authority for this repository would be:

```text
map --(SLAM/global localization)--> odom
odom --(continuous odometry/EKF)--> base_link
base_link --(robot_state_publisher)--> links and sensors
```

| Transform | Recommended sole owner |
|---|---|
| `map -> odom` | Global localization/SLAM correction layer |
| `odom -> base_link` | EKF or simulation ground-truth odometry source |
| `base_link -> articulated/fixed links` | `robot_state_publisher` |

Ground-truth mode implements `odom -> base_link` plus an identity `map -> odom`. SLAM mode uses two filters because a single filter with `world_frame: odom` cannot also produce `map -> odom`.

## Remaining findings and risks

| Severity | Finding | Impact |
|---|---|---|
| High | No `/odom_lko` publisher exists in the current gait controller | SLAM simulation temporarily depends on Gazebo ground-truth local odometry |
| Medium | Depth is stamped `camera_link`, RGB is stamped `camera_link_optical` | Easy to misinterpret depth point axes; must verify bridge output convention |
| Medium | Global EKF behavior has not yet been verified in a running ROS container | Runtime timestamps and transform availability may require tuning |
| Medium | Gazebo pose conversion currently provides pose but no twist | The local EKF derives motion from successive poses rather than receiving ground-truth velocity |
| Low | Documentation says the vehicle contract is `/tf: base_link -> odom` | TF notation is reversed; the conventional transform is `odom -> base_link` |

## Remaining work

1. Implement `/odom_lko` and replace `/odom_ground_truth` in `ekf_local.yaml` for hardware operation.
2. Verify the dual-filter stack at runtime with `tf2_tools view_frames`, `tf2_echo map base_link`, and `tf2_echo odom base_link`.
3. Confirm the Gazebo depth point cloud's axis convention and change its frame ID to `camera_link_optical` if required.
4. Tune EKF covariances using recorded outdoor data.

## Source map

- `src/quadruped_gazebo/launch/gazebo.launch.py` — localization mode and Gazebo pose bridge.
- `src/quadruped_gazebo/scripts/gazebo_ground_truth_odometry.py` — canonical simulation odometry and optional TF.
- `src/quadruped_gazebo/urdf/quadruped.urdf.xacro` — robot link tree, sensor extrinsics, and Gazebo sensor frame IDs.
- `src/localization/src/orbslam3_rgbd_node.cpp` — SLAM odometry frames and optional TF publication.
- `src/localization/config/ekf_local.yaml` — local `odom -> base_link` filter.
- `src/localization/config/ekf_global.yaml` — global `map -> odom` filter.
- `src/localization/launch/localization.launch.py` — active SLAM/EKF TF ownership choice.
- `src/elevation_mapping_cupy_pkg/simple_elevation_mapping_cupy/launch/elevation_mapping.launch.py` — active mapping frame overrides.
- `src/elevation_mapping_cupy_pkg/simple_elevation_mapping_cupy/config/elevation_mapping.yaml` — active mapper frame defaults.
- `src/elevation_mapping_cupy_pkg/repo/elevation_mapping_cupy/scripts/elevation_mapping_node.py` — TF lookups performed for clouds and robot pose.
- `src/nav2_mppi_navigation/nav2_mppi_navigation/config/nav2_mppi_params.yaml` — Nav2 global, odometry, and robot frames.
- `src/nav2_mppi_navigation/nav2_mppi_navigation/launch/nav2_mppi.launch.py` — Nav2 launch argument wiring.
- `src/video_camera_sim/video_camera_sim/video_node.py` and `src/monocular_depth/monocular_depth/depth_node.py` — perception message frame propagation.
