# odometry_scale_estimation

Compares two `nav_msgs/Odometry` streams over the last few seconds and
publishes their relative translation scale — the kinetic probe for the
monocular depth **scale hypothesis via motion** (Alternative 2, next to the
ground-plane probe of Alternative 1): KISS-ICP running on a uniformly scaled
point cloud estimates a uniformly scaled motion, so comparing its odometry
against the metric leg-kinematics + IMU odometry recovers the depth scale
`s` — no ground-truth depth image, no ground plane in view required.

Why chords are the observable: the straight-line displacement between two
instants of the same rigid body has the same length in any odometry frame,
so the two streams need no extrinsic alignment — only time alignment (from
message stamps). Chords also avoid the upward bias that trajectory noise
adds to integrated path length.

## Pipeline

```
/odom_lko  (reference: metric truth) ──┐
                                       ├─► sliding window ─► matched chords ─► fit
/odom_lidar (estimate: carries s)   ───┘
                                          │
   /odom_scale/scale        std_msgs/Float32   s in chord_est ≈ s·chord_ref (+ b)
   /odom_scale/shift        std_msgs/Float32   b (affine model only, see below)
   /odom_scale/residual     std_msgs/Float32   fit RMS (m) — hypothesis quality
   /odom_scale/ref_motion   std_msgs/Float32   longest reference chord (m)
   /odom_scale/marker       MarkerArray        RViz: overlay + motion arrows
```

The marker carries two visual comparisons: the two window trajectories
rigidly overlaid in `marker_frame`, and two motion-vector ARROWs from the
`base_link` origin — each stream's displacement over the window, rotated
into the body frame by that stream's own attitude. Arrow length is the
motion magnitude, so the scale error is the length ratio (green =
reference, orange = estimate); an angle between the arrows is a heading
disagreement between the streams.

One node per estimate topic; the launch file takes a comma-separated
`estimate_topics` list and namespaces each estimator as
`<output_namespace>/<key>/*` (key from the topic), all sharing one
`reference_topic` — so several odometry sources can be probed side by side.

## Fit models (`model` parameter)

| Model | Fit | When |
|---|---|---|
| `scalar` | `chord_est = s·chord_ref`, least squares through the origin | Working hypothesis: monocular depth error is a pure per-window scale. |
| `affine` | `chord_est = s·chord_ref + b`, least squares with intercept | Suspected fallback. Needs spread in chord lengths (mixed pair separations provide it); windows without spread are rejected, not answered arbitrarily. |

**`shift` semantics — read before wiring it anywhere.** `b` lives in
*displacement* space: it absorbs the estimate stream's noise floor (ICP
jitter adds a near-constant offset to every chord) and is a *diagnostic*
for a non-scalar depth error. It is **not** numerically the depth-space
shift of `pointcloud_scale_correction`'s affine model (`depth_pred =
s·depth_true + b`), because an affine depth error does not distort ICP
motion as a clean chord-affine map. Interpretation:

- `scale` stable and `shift ≈ 0` → scalar hypothesis supported; feed
  `/odom_scale/scale` to `pointcloud_scale_correction` as-is.
- `shift` consistently non-zero, or `scale` drifting with scene depth →
  affine depth error suspected; a true depth-space `(s, b)` split then
  needs two observables (this probe + the ground-plane probe jointly).

## Run

Gazebo sanity check (both streams metric — scale should sit at 1.0):

```bash
docker compose up -d --build quadruped_leg_odometry lidar_odometry odometry_scale_estimation
ros2 topic echo /odom_scale/scale
```

The actual depth probe — point KISS-ICP at the monocular depth cloud so its
odometry inherits the depth scale:

```bash
LIDAR_POINTCLOUD_TOPIC=/depth/points \
docker compose up -d --build quadruped_leg_odometry lidar_odometry odometry_scale_estimation
rqt_plot /odom_scale/scale/data /odom_scale/residual/data /odom_scale/ref_motion/data
```

Drive the robot around (`tmux attach -t quadruped_test_controller`) —
`ref_motion` shows whether there is enough excitation for the estimate to
mean anything; nothing is published while the robot stands still.

## Key parameters

| Parameter | Default | Meaning |
|---|---|---|
| `reference_topic` | `/odom_lko` | Metric-truth odometry (leg kinematics + IMU) |
| `estimate_topic` | `/odom_lidar` | Scale-bearing odometry (node-level; launch takes `estimate_topics`) |
| `model` | `scalar` | `scalar` or `affine` |
| `window_duration` | 5.0 s | Sliding comparison window |
| `min_pair_dt` | 0.5 s | Minimum time separation of a chord pair |
| `min_motion` | 0.1 m | Minimum reference chord before fitting (excitation gate) |
| `min_spread` | 0.05 m | Affine only: minimum std of reference chords |
| `max_position_jump` | 1.0 m | Larger step ⇒ odometry re-initialised ⇒ buffer restarts |
| `max_interp_gap` | 0.5 s | Never interpolate the reference across a longer dropout |
| `time_offset` | 0.0 s | Constant stamp lag of the estimate stream |
| `publish_rate` | 2.0 Hz | Output cadence; each cycle re-fits the whole window |
| `marker_frame` / `base_frame` | `odom` / `base_link` | RViz frames: trajectory overlay / motion-vector arrow origin |

Estimates are published raw per window — smoothing, gating and staleness
policy live in the consumer (`pointcloud_scale_correction`), same contract
as the ground-plane probe.

## Tests

`test/test_trajectory_compare.py` runs on the host (pure numpy): buffer
pruning/restart detection, gap-aware interpolation, scalar/affine recovery,
excitation and spread gates, ICP-jump gating, Kabsch alignment for the
marker. `test/test_node_smoke.py` needs a ROS environment: two synthetic
odometry streams ×2 apart in → `/odom_scale/scale ≈ 2.0` out within 10 s.

## Caveats

- **Both streams must track the same rigid body point** (`base_link` under
  the vehicle interface contract). An odometry reporting the camera frame
  instead picks up lever-arm motion under rotation.
- **The reference's own error is the probe's floor.** LKO slip on loose
  terrain reads as a scale change; cross-check against `/odom_scale/residual`
  and, when a real lidar is present, against a `/odom_lidar` reference.
- **Pure rotation is unobservable** — chords need translation. The
  excitation gate (`min_motion`) holds the output rather than publishing
  noise.
- **Time alignment matters more as speed grows**: a stamp lag of Δt at speed
  v biases chords by ~v·Δt. Use `time_offset` for a known constant lag.
