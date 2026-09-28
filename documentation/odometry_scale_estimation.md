# Odometry Scale Estimation — Detailed Reference

## Why this module exists

This is the second of the two planned "solve depth kinetically" probes for
the monocular depth scale problem:

- **Alternative 1 (`ground_plane_estimation`):** ground-plane distance vs.
  known camera height. Needs a visible, locally flat ground patch and a
  height reference.
- **Alternative 2 (this module, `src/odometry_scale_estimation/`):** compare
  the motion reported by the proprioceptive metric odometry (leg kinematics
  + IMU, `quadruped_leg_odometry`) against the motion inferred from the
  monocular point cloud over the last few seconds.

The insight that makes Alternative 2 work without any new estimator: ICP on
a **uniformly scaled world estimates a uniformly scaled motion**. If the
monocular depth error is a scale factor (`depth_pred ≈ s · depth_true`),
every point cloud is `s ×` the true scene, and KISS-ICP (`lidar_odometry`,
which accepts *any* `PointCloud2`) fed those clouds reports translations
that are exactly `s ×` the true translations. The leg-kinematics odometry
reports the true (metric) translations. The ratio of the two motions is `s`
— per window, with no ground-truth depth image, no ground plane in view,
and no assumption about scene content beyond "ICP can track it."

```
scale_estimate = ||displacement seen by monodepth-ICP|| /
                 ||displacement seen by legs + IMU||
```

The two probes are complementary: the ground-plane probe needs no motion
but needs flat ground in view; this probe needs motion but no particular
scene geometry. Where both run, they cross-check each other — and jointly
they can separate a scale from a shift (see "Affine" below).

---

## The chord argument

Comparing two odometry streams directly by position is wrong: each stream
has its own arbitrary origin and orientation (`odom` at t₀ for LKO, first
scan for KISS-ICP), and aligning them adds an extrinsic estimation problem.

Instead the module compares **chords** — straight-line displacements
`||p(t₂) − p(t₁)||` between two instants. A chord is the distance between
two poses of the *same rigid body*, so its length is invariant to which
frame the trajectory is expressed in. Two streams tracking the same body
point produce chords that differ only by their respective errors — and by
`s`, when one stream rides on scaled depth. No extrinsic alignment needed;
only *time* alignment, which comes from message stamps (the reference is
linearly interpolated at the estimate's stamps; `time_offset` absorbs a
known constant lag).

Chords are also preferred over integrated path length: the path length of a
noisy trajectory is biased upward by the noise itself (every jitter adds
distance), while a chord between two sufficiently separated samples is not.

Within the window, every pair of samples at least `min_pair_dt` apart
contributes one chord pair `(chord_ref, chord_est)` — mixed separations on
purpose, since long chords carry the best signal-to-noise for `s` while the
short ones expose the noise floor (which is what the affine intercept
captures).

---

## Pipeline

```
/odom_lko   (reference — metric truth)   ─┐  TrajectoryBuffer (window,
                                          │  restart + jump detection)
/odom_lidar (estimate — carries s)       ─┘
        │
        ▼
   time alignment: reference interpolated at estimate stamps
        │            (gap-aware — never interpolates across a dropout)
        ▼
   chord pairs (all sample pairs ≥ min_pair_dt apart, thinned to a cap)
        │
        ▼
   per-pair ratio gate (ICP jumps / glitches) + excitation gate
        │
        ▼
   fit model (trajectory_compare.py, pure numpy)
        │  scalar: chord_est = s·chord_ref        (LS through origin)
        │  affine: chord_est = s·chord_ref + b    (LS with intercept)
        ▼
   /odom_scale/scale        s                  ──▶ pointcloud_scale_correction
   /odom_scale/shift        b (affine only)
   /odom_scale/residual     fit RMS (m)
   /odom_scale/ref_motion   longest reference chord (m)
   /odom_scale/marker       trajectory overlay + motion arrows (RViz)
```

One `OdomScaleNode` instance compares one estimate stream against the
shared reference. The launch file (`odom_scale.launch.py`) takes a
comma-separated `estimate_topics` list and spawns one node per topic under
`<output_namespace>/<key>/*` — same multi-channel pattern as the
ground-plane probe.

---

## The two fit models

Model choice is a parameter (`model:=scalar|affine`), mirroring
`pointcloud_scale_correction`'s pluggable error models: the scalar model is
the working hypothesis, the affine model the suspected fallback, and
switching costs a config change. Adding a further model (e.g. a robust or
windowed-Umeyama variant) is one `ScaleModel` subclass registered in
`MODELS` — the node never sees the math.

### Scalar

Least squares through the origin over the gated chord pairs:

```
s = Σ (chord_ref · chord_est) / Σ (chord_ref²)
```

Long chords dominate the sum — exactly where the signal-to-noise is best.
The residual RMS (`/odom_scale/residual`) is the per-window hypothesis
check: a genuinely scalar error leaves residuals at the odometry noise
level; a structured error inflates them.

### Affine

Ordinary least squares with an intercept: `chord_est = s·chord_ref + b`.
Fitting an intercept requires **spread** in the reference chord lengths
(all-equal chords make `s` and `b` indistinguishable) — mixed pair
separations provide it naturally, and windows with `std(chord_ref) <
min_spread` are rejected rather than answered with an arbitrary split.

**What `b` is — and is not.** `b` lives in displacement space. It absorbs:

- the estimate stream's noise floor: ICP jitter of magnitude σ adds
  ~`√(2)·σ` to every chord regardless of length — a textbook intercept;
- the first-order signature of a non-scalar depth error: an affine depth
  error (`depth_pred = s·depth_true + b_depth`) warps each cloud along its
  view rays, which is *not* rigid-consistent between viewpoints, so ICP's
  best-fit motion picks up geometry-dependent bias that shows up here as a
  non-zero intercept and/or an `s` that drifts with scene depth.

It is therefore a *diagnostic*, not (numerically) the depth-space
`b_depth` that `pointcloud_scale_correction`'s affine model divides out.
Recovering a true `(s, b_depth)` pair needs a second independent observable
— which is precisely what running this probe *and* the ground-plane probe
simultaneously provides: the ground-plane scale reads `s + b_depth/h·(…)`
at camera-height distances while the motion scale reads `s` at
scene-traversal distances, so a consistent gap between the two probes'
scale outputs is the shift's fingerprint.

### Reading the results

| Observation | Conclusion |
|---|---|
| `scale` stable, `residual` low, affine `shift ≈ 0` | Scalar hypothesis holds; the scale value is directly usable as a correction divisor. |
| `shift` consistently non-zero, or `scale` drifts with scene / speed | Affine (or worse) depth error; correlate with the ground-plane probe before trusting either single number. |
| `scale ≈ 1` when KISS-ICP runs on a *real* lidar | Sanity check of the whole probe: two metric streams must agree. Also a live health check on LKO slip. |
| `ref_motion` near zero | The robot is standing; the probe is (correctly) silent. Not a failure. |

---

## Gates — why the output is sometimes silent

Publishing nothing beats publishing noise; every gate below holds the
output rather than degrading it:

| Gate | Parameter | Rejects |
|---|---|---|
| Excitation | `min_motion` (0.1 m) | Standing-still windows: chords made of pure noise, ratio undefined. |
| Pair ratio | `min_scale`/`max_scale` (0.02/50) | Individual ICP jumps or interpolation glitches (pairs with reference chords under `min_chord` bypass it — their ratio is meaningless but they carry the noise floor and get ~zero weight in the scalar fit). |
| Pair count | `min_pairs` (3) | Windows where gating left too little to fit. |
| Spread (affine) | `min_spread` (0.05 m) | Ill-conditioned intercept fits. |
| Interpolation gap | `max_interp_gap` (0.5 s) | Fabricating reference motion across a dropout. |
| Stream restart | `max_position_jump` (1.0 m), backwards stamps | Chords spanning an odometry re-initialisation (KISS-ICP restart, bag loop) — the buffer clears and refills. |
| Staleness | `stale_timeout` (5 s) | Fitting a window that stopped updating (a stream died). |

`/odom_scale/ref_motion` is published every cycle *even when the fit is
not*, so a silent `scale` topic is distinguishable from a dead node: flat
`ref_motion ≈ 0` reads "standing", not "broken".

---

## The RViz marker

`/odom_scale/marker` carries two visual comparisons — the visual-inspection
counterpart to the numeric topics, in the spirit of the depth comparison
workflow. Green is always the reference stream, orange the estimate.

**Trajectory overlay** (LINE_STRIPs, in `marker_frame`): both window
trajectories, with the estimate window rigidly aligned onto the reference
via Kabsch (rotation + translation, **never scale** — the alignment must
not absorb the observable). A scale error is directly visible as the
orange track outrunning or undershooting the green track along the same
shape.

**Motion-vector arrows** (ARROWs, from the `base_frame` origin): each
stream's displacement over the window — `p(t_end) − p(t_start)` — rotated
into the body frame by that stream's *own* latest attitude
(`nav_msgs/Odometry` pose orientation is exactly the body→odom rotation),
so both arrows originate at the robot centre without any cross-stream
extrinsic. The arrow's direction and length *are* the motion vector
(start/end-point form, no orientation quaternion): the scale error is the
length ratio between orange and green, and an angle between the arrows is
a heading disagreement between the two odometries — each observable stays
visible on its own axis. Displacements under 1 mm get no arrow (a
standing robot shows the trajectory dots, not noise needles).

---

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `reference_topic` | `/odom_lko` | Metric-truth `nav_msgs/Odometry`; shared across instances |
| `estimate_topic` | `/odom_lidar` | Scale-bearing odometry (launch-level arg is `estimate_topics`, comma-separated) |
| `output_namespace` | `/odom_scale` | Parent of `scale`/`shift`/`residual`/`ref_motion`/`marker` |
| `model` | `scalar` | `scalar` or `affine` |
| `window_duration` | 5.0 s | Sliding comparison window ("the last few seconds") |
| `min_pair_dt` | 0.5 s | Minimum time separation of a chord pair |
| `max_samples` | 40 | Even thinning cap before O(n²) pairing |
| `max_interp_gap` | 0.5 s | Reference interpolation validity across dropouts |
| `max_position_jump` | 1.0 m | Step size that means "odometry re-initialised" |
| `time_offset` | 0.0 s | Constant stamp lag of the estimate stream (added to its stamps) |
| `min_motion` | 0.1 m | Excitation gate |
| `min_pairs` | 3 | Minimum gated pairs for a fit |
| `min_scale` / `max_scale` | 0.02 / 50.0 | Per-pair ratio gate |
| `min_chord` | 0.01 m | Reference chords below this bypass the ratio gate |
| `min_spread` | 0.05 m | Affine only: minimum std of reference chords |
| `publish_rate` | 2.0 Hz | Output cadence; each cycle re-fits the whole window |
| `stale_timeout` | 5.0 s | Skip when the newest common sample is older; 0 disables |
| `publish_marker` / `marker_frame` | true / `odom` | RViz marker and the frame the reference poses live in (trajectory overlay) |
| `base_frame` | `base_link` | Robot body frame the motion-vector arrows originate from |

---

## Topics

| Topic | Type | Notes |
|---|---|---|
| `<ns>/scale` | `std_msgs/Float32` | `s` — same consumer contract as `/ground_plane/scale`; `pointcloud_scale_correction` can subscribe either. Raw per-window; smoothing is the consumer's job. |
| `<ns>/shift` | `std_msgs/Float32` | `b`, affine model only. Displacement-space diagnostic — see above before treating it as a depth shift. |
| `<ns>/residual` | `std_msgs/Float32` | Fit RMS (m) over the gated pairs. |
| `<ns>/ref_motion` | `std_msgs/Float32` | Longest reference chord in the window (m). Published every cycle. |
| `<ns>/marker` | `visualization_msgs/MarkerArray` | Two LINE_STRIPs (trajectory overlay in `marker_frame`) + two ARROWs (window motion vectors from the `base_frame` origin). Reference green, estimate orange. |

---

## Running it

**Sanity run** (Gazebo, KISS-ICP on the ground-truth depth cloud — both
streams metric, scale must sit at 1.0):

```bash
LIDAR_POINTCLOUD_TOPIC=/depth_camera/points \
docker compose up -d --build quadruped_leg_odometry lidar_odometry odometry_scale_estimation
ros2 topic echo /odom_scale/scale
```

**The depth probe** (KISS-ICP on the *monocular* cloud — its odometry
inherits the depth scale):

```bash
LIDAR_POINTCLOUD_TOPIC=/depth/points \
docker compose up -d --build quadruped_leg_odometry lidar_odometry odometry_scale_estimation
rqt_plot /odom_scale/scale/data /odom_scale/residual/data /odom_scale/ref_motion/data
```

Drive the robot with the keyboard teleop and compare against the
ground-plane probe's `/ground_plane/scale` on the same run — agreement
between two independent observables is the strongest evidence the scalar
hypothesis allows; disagreement is the affine fingerprint.

**Closing the loop** — feed the estimate into the corrector:

```bash
SCALE_CORRECTION_SCALE_TOPICS=/odom_scale/scale \
docker compose up -d --build pointcloud_scale_correction
```

---

## Caveats

- **Both streams must track the same rigid body point** (`base_link` per
  the vehicle interface contract; `lidar_odometry` conjugates its poses to
  `base_link` via TF, LKO publishes `base_link` natively). An odometry
  tracking the camera origin instead picks up lever-arm translation under
  body rotation, which reads as scale noise.
- **The reference's own error is the probe's floor.** LKO drifts on
  slip-prone terrain; over a 5 s window that drift is small but not zero,
  and it enters the ratio directly. The drift-model covariance LKO
  publishes is not (yet) consumed here — a weighted fit is a natural
  extension (`ScaleModel` subclass).
- **Pure rotation is unobservable.** Chords need translation; turning in
  place keeps the probe silent by design.
- **A per-frame *varying* scale is averaged.** The window fit recovers the
  effective scale over the last few seconds. If `s` genuinely varies
  per-frame (the ground-plane probe can see that; this one cannot), shrink
  the window at the cost of noise, or treat this probe's output as the
  smoothed component.
- **KISS-ICP on monodepth clouds is itself an experiment.** Monocular
  clouds are denser, noisier, and shorter-range than lidar scans; expect to
  drop `LIDAR_MAX_RANGE` and tune the voxel size. If ICP loses tracking the
  jump/restart guards keep the probe honest, but the duty cycle drops with
  it.

---

## Source layout

| File | Role |
|---|---|
| `odometry_scale_estimation/trajectory_compare.py` | Buffers, chord pairing, scalar/affine models, Kabsch alignment — pure numpy, ROS-free |
| `odometry_scale_estimation/odom_scale_node.py` | ROS node: stream intake, windowing, publishing |
| `launch/odom_scale.launch.py` | Spawns one node per `estimate_topics` entry, keyed by topic name |
| `test/test_trajectory_compare.py` | Host-runnable: buffer restart/gap handling, scalar/affine recovery, gate behaviour, alignment properties |
| `test/test_node_smoke.py` | ROS-environment: two synthetic odometry streams ×2 apart → `/odom_scale/scale ≈ 2.0` within 10 s |
| `README.md` | Module-local quick reference (parameters, run commands) |
