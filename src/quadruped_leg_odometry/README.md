# quadruped_leg_odometry

Leg-kinematics + IMU odometry (**LKO**). One module, two sub-components:

- **IMU attitude** (`imu_attitude.py`) — uses the orientation quaternion from
  `/imu/data` when the source provides one (Gazebo does). Falls back to a
  Mahony-style complementary filter (gyro integration + accelerometer tilt
  correction) for real ICM-42688-class hardware that outputs only rates and
  specific force. Yaw is gyro-only in that mode and drifts.
- **Leg odometry** (`kinematics.py` + `leg_odometry_node.py`) — forward
  kinematics and geometric Jacobians for all four legs from `/joint_states`.
  Stance feet are detected kinematically (feet within a height band of the
  lowest foot, measured along gravity). Each stance foot is stationary in the
  world, so `v_base = -(ω × p_foot + J q̇)`; the estimates are averaged and
  integrated with the IMU attitude into an odometry pose.

```
/joint_states ──► FK + Jacobians ─┐
/imu/data     ──► attitude, ω ────┴─► /odom_lko  (nav_msgs/Odometry)
```

## Interface

| Topic | Type | Direction |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | in |
| `/imu/data` | `sensor_msgs/Imu` | in |
| `/odom_lko` | `nav_msgs/Odometry` | out (`odom` → `base_link`, twist in `base_link`) |

- **No TF by default.** `odom→base_link` belongs to the robot_localization EKF
  (or the sim ground-truth publisher). `publish_tf:=true` is for standalone
  debugging only — never alongside another publisher of the same edge.
- **Covariances are meaningful.** Twist covariance is the per-sample estimate
  (inflated ×100 when fewer than two feet are in stance, where the lever arm
  is unobservable). Pose covariance grows with traveled distance and turned
  angle — integrated leg odometry drifts; fuse the twist, not the pose.
- Assumes `imu_link` is co-located and axis-aligned with `base_link`
  (true in the URDF: `imu_joint` origin is identity).

## Run

```bash
# In the container (started via root docker-compose)
ros2 launch quadruped_leg_odometry leg_odometry.launch.py use_sim_time:=true

# Standalone debug with its own TF (nothing else on odom->base_link!)
ros2 launch quadruped_leg_odometry leg_odometry.launch.py publish_tf:=true
```

Environment variables (compose): `USE_SIM_TIME`, `JOINT_STATES_TOPIC`,
`IMU_TOPIC`, `ODOM_LKO_TOPIC`, `LKO_PUBLISH_TF`.

## Validation against ground truth (sim)

```bash
ros2 topic echo /odom_lko --field twist.twist.linear
ros2 topic echo /odom_ground_truth --field twist.twist.linear
```

Drive with the keyboard controller and compare integrated pose drift:
LKO x/y should track ground truth within a few % of distance traveled on
flat ground; slip and stance-transition errors accumulate as drift.

## Known limits

- Feet near touchdown/liftoff can fall inside the stance band while still
  moving; their world velocity pollutes the average slightly. Acceptable at
  trot; revisit with the URDF foot-contact sensors (`FL_foot_contact` etc.,
  not yet bridged to ROS) if drift is dominated by transitions.
- Robot geometry constants are duplicated from the URDF/gait controller
  (`kinematics.py` — must match URDF properties).
