# Ground Plane Estimation — Detailed Reference

## Why this module exists

Depth Anything V2 (DA-V2) has a scale problem: its metric depth output does not
match real-world distances consistently. Before building a full quantitative
comparison harness, the working hypothesis is the simplest one —

> Monocular depth error is (mostly) a **scale factor**: `depth_pred ≈ s · depth_true`,
> with `s` varying per frame / per scene.

If that holds, we don't need ground-truth depth to measure `s`. We need one
thing whose *true* metric value we already know: **how far the camera is above
the ground.** Fit the ground plane in the predicted point cloud, measure the
camera's perpendicular distance to it, divide by the known true height, and
`s` falls out — no ground-truth depth image required.

This module (`src/ground_plane_estimation/`) is that measurement device. It is
the first of two planned "solve depth kinetically" probes:

- **Alternative 1 (this module):** ground-plane distance vs. known camera height.
- **Alternative 2 (`odometry_scale_estimation`):** compare the IMU +
  leg-odometry motion estimate against the motion implied by the monocular
  point cloud over time — see `documentation/odometry_scale_estimation.md`.

---

## The geometric argument

A monocular depth model with a pure per-frame scale error produces a point
cloud that is a uniformly scaled copy of the true cloud, in the **camera
frame**:

```
p_pred = s · p_true          (both expressed in camera_optical)
```

To get a distance in `base_link` we apply the camera→base rigid transform:
rotation `R` (attitude of the camera relative to the body, always exact — it
comes from the URDF, not the depth network) and translation `t` (the true,
correctly-known camera offset from `base_link`):

```
p_base_pred = R · p_pred + t = R · (s · p_true) + t
```

If you now measure the **base_link-to-plane distance**, the answer mixes a
scaled term (`R·s·p_true`) with an un-scaled term (`t`). That distance does
not scale cleanly with `s` — it's not a clean observable.

But look at the distance from the **camera origin** instead. The camera
origin's position in `p_base_pred` terms is just `t` (the camera is at its own
origin in its own frame). The plane, expressed via the fitted normal `n` and
offset `d` in the base frame, gives a signed distance to any point `q` as
`n·q + d`. Because rotation preserves distances and the plane fit is done
directly on the *predicted* cloud, the perpendicular distance from the camera
origin to the plane fitted from `p_pred` is:

```
camera_height_measured = s · camera_height_true
```

exactly, with no `t` contamination — the translation `t` is what's used to
locate the camera origin in the base frame in the first place, not something
riding along with the scale error. So:

```
scale_estimate = camera_height_measured / camera_height_true_expected
```

is a direct, per-frame estimate of `s`. This is what `/ground_plane/scale`
publishes. `/ground_plane/distance` (the `base_link`-to-plane distance) is
published too, but only for plotting/sanity — it is **not** the scale
observable, per the derivation above.

---

## Pipeline

```
PointCloud2 (camera_optical, or any frame)
        │
        ▼
   TF → base_frame            (rotation + translation from the URDF, exact)
        │
        ▼
   range gate + low-z percentile filter
        │             (keeps ground candidates without assuming a known
        │              scale — an absolute height cutoff would fail
        │              whenever s is far from 1)
        ▼
   RANSAC plane fit (plane_fit.py, pure numpy)
        │  - reject hypotheses whose normal is > max_ground_angle_deg
        │    from base +z (rejects walls / obstacles as "ground")
        │  - two-round least-squares refinement (SVD) on the consensus set
        ▼
   signed distance from base_link origin  ──▶ /ground_plane/distance
   signed distance from camera origin     ──▶ /ground_plane/camera_height
   camera_height / expected_camera_height ──▶ /ground_plane/scale
   plane visualization                    ──▶ /ground_plane/marker
```

One `GroundPlaneNode` instance handles exactly one input point cloud topic.
The launch file (`ground_plane.launch.py`) takes a comma-separated
`input_topics` list and spawns one node per topic, each namespaced by a key
derived from the topic name — so it plugs directly into the depth comparison
module's per-backend channels:

```
/depth_compare/gt/points            ─▶ /ground_plane/gt/scale
/depth_compare/dav2_vkitti/points   ─▶ /ground_plane/dav2_vkitti/scale
/depth_compare/dav2_hypersim/points ─▶ /ground_plane/dav2_hypersim/scale
/depth_compare/metric3d_s/points    ─▶ /ground_plane/metric3d_s/scale
```

All channels run on the same underlying frame (depth_comparison's own
guarantee — see its README), so the four scale numbers are directly
comparable at every instant.

---

## The "expected camera height" reference

`scale = camera_height_measured / camera_height_expected` needs a true value
for the denominator. Two mutually exclusive sources, selected by parameter:

| Source | Parameter | How it works | When to use |
|---|---|---|---|
| Static | `expected_base_height` (> 0) | `expected = expected_base_height + camera_z_offset_in_base` (camera z offset comes from the exact URDF TF). Assumes a level body. | Real hardware, or any run where the robot's standing height is known and roughly constant. |
| TF-driven | `height_reference_frame` (non-empty) | `expected = z` of the cloud frame's origin, looked up in the reference frame, per-frame. | Gazebo with `LOCALIZATION_MODE=ground_truth`, where `odom`'s z=0 plane *is* the true ground by construction. |

`expected_base_height` wins if both are set. If neither is set, `/scale` is
simply not published — `/distance` and `/camera_height` still are, useful for
manual before/after comparison.

The depth-comparison profile (`depth-comparison.env`) sets
`GROUND_PLANE_HEIGHT_REFERENCE_FRAME=odom`, which is valid specifically
*because* `LOCALIZATION_MODE=ground_truth` makes `odom` a flat, driftless
world frame with ground at z=0. This reference stops being valid the moment
the terrain isn't flat, or SLAM/EKF drift is present (offline video or
`LOCALIZATION_MODE=slam` runs) — see Caveats.

---

## RANSAC plane fit (`plane_fit.py`)

Kept dependency-free (pure numpy, no ROS/tf2 imports) so it's unit-testable on
the host without a ROS install.

1. **Candidate gate** (done by the node, before calling `fit_ground_plane`):
   range-gate in the cloud frame, then keep the lowest `candidate_z_percentile`
   of points by base-frame z. Percentile-based rather than an absolute height
   threshold, because an absolute cutoff (e.g. "keep points below 0.3 m")
   silently breaks the moment `s` deviates from 1 — the whole point of this
   module is to handle unknown `s`.
2. **RANSAC hypotheses**: sample 3 random points, form the plane through them,
   reject if the normal is more than `max_angle_rad` from `up` (default 35°,
   filters out walls/obstacles being mistaken for ground), count inliers
   within `threshold` (default 3 cm).
3. **Refinement**: two rounds of SVD least-squares fit on the current inlier
   set, re-selecting inliers against the refined plane each round. This
   pulls the plane off the exact 3-point sample and onto the full consensus
   set, and drops points RANSAC's coarse threshold let through.
4. **Rejection conditions**: too few points overall, no hypothesis clears
   `min_inliers`, or the refined normal exceeds the angle gate — all return
   `None` and the node logs a throttled warning rather than publishing a
   bogus distance.

`PlaneFit.signed_distance(point)` returns `n·point + d` — positive on the
`up` side (robot standing above ground), which is what both `/distance` and
`/camera_height` publish directly (no `abs()`), so a plane fit gone wrong
(sign flipped, robot below the fitted plane) is visible as a negative reading
rather than silently rectified.

---

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `input_topic` | `/depth_camera/points` | PointCloud2 to fit (per-node; launch-level arg is `input_topics`, comma-separated) |
| `output_namespace` | `/ground_plane` | Parent of `distance`/`camera_height`/`scale`/`marker` |
| `base_frame` | `base_link` | Distance reference frame; TF `base_frame ← cloud_frame` must resolve (identity skipped if the cloud already arrives in this frame) |
| `target_fps` | 5.0 | Upper bound on processing rate; excess clouds are dropped, not queued |
| `min_range` / `max_range` | 0.2 / 8.0 | Range gate in the cloud frame, cloud units (i.e. pre-scale-correction) |
| `candidate_z_percentile` | 60.0 | Keep only this fraction (by base-frame z, lowest first) as RANSAC candidates |
| `max_points` | 4000 | Subsample cap on candidates before RANSAC, for latency |
| `ransac_iterations` | 120 | RANSAC hypothesis count |
| `ransac_threshold` | 0.03 m | Inlier distance threshold |
| `max_ground_angle_deg` | 35.0 | Reject plane hypotheses steeper than this vs. base +z |
| `min_inliers` | 100 | Reject fits with fewer supporting points |
| `expected_base_height` | 0.0 | Static scale reference (m); > 0 enables it and takes priority |
| `height_reference_frame` | `''` | TF-driven scale reference; used when `expected_base_height` is 0 |
| `publish_marker` | true | Publish the RViz plane slab |

---

## Topics

| Topic | Type | Frame | Notes |
|---|---|---|---|
| `<ns>/distance` | `sensor_msgs/Range` | `base_frame` | `base_link`-to-plane distance. Plotting/sanity only — **not** the scale observable (see derivation above). |
| `<ns>/camera_height` | `sensor_msgs/Range` | cloud's `frame_id` | Camera-origin-to-plane distance. This *is* the scale observable. |
| `<ns>/scale` | `std_msgs/Float32` | — | `camera_height / expected_camera_height`. Only published when a height reference is configured. |
| `<ns>/marker` | `visualization_msgs/Marker` (CUBE) | `base_frame` | Thin (1 cm) slab aligned to the fitted plane, 2×2 m, for RViz sanity-checking the fit visually. |

`Range.range` carries the signed distance (can be read as negative if the
robot ends up "under" the fitted plane — a fit-quality red flag, not clamped
away).

---

## Running it

**Standalone against Gazebo ground-truth depth** (sanity check — should track
`s ≈ 1.0`):

```bash
docker compose up -d --build ground_plane_estimation
ros2 topic echo /ground_plane/camera_height
```

**Against every depth-comparison channel at once** (the actual experiment):

```bash
docker compose --env-file depth-comparison.env \
  -f docker-compose-depth-comparison.yaml up -d --build
# ground_plane_estimation is included in this compose entry point already,
# with GROUND_PLANE_INPUT_TOPICS pre-wired to the four comparison channels
# and GROUND_PLANE_HEIGHT_REFERENCE_FRAME=odom.

rqt_plot \
  /ground_plane/gt/scale/data \
  /ground_plane/dav2_vkitti/scale/data \
  /ground_plane/dav2_hypersim/scale/data \
  /ground_plane/metric3d_s/scale/data
```

Drive the robot with the keyboard teleop (`tmux attach -t
quadruped_test_controller`) to sample the scale estimate across viewpoints and
distances — a real per-frame-constant scale error should show up as a flat
`gt` line near 1.0 and roughly-flat-but-offset lines for each monocular model.

---

## Reading the results — what confirms or kills the hypothesis

- **Hypothesis holds**: `gt/scale` stays near 1.0 (sanity), and each
  monocular model's `scale` is roughly *constant* across frames/viewpoints,
  just offset from 1.0 by a per-model factor. That constant is directly
  usable as a correction multiplier.
- **Hypothesis is wrong / incomplete**: a model's `scale` *drifts* with scene
  content, distance to ground, or viewing angle — that's evidence of an
  affine error (`depth_pred ≈ s·depth_true + b`, a shift term as well as
  scale) rather than pure scale. A shift term bends the plane instead of
  moving it uniformly, which should also show up as elevated `rms` in the
  node's fit — cross-check by watching for `no ground plane found` /
  fit-rejection warnings correlating with scale swings.
- **Fit failing outright** (frequent "no ground plane found" warnings): either
  the candidate gate isn't reaching the floor (check
  `candidate_z_percentile` against the model's actual scale error — a model
  with `s` far from 1 needs the gate loose enough to still contain ground
  points) or the scene genuinely lacks a flat patch in view.

---

## Caveats

- **Local plane, not terrain-following clearance.** On uneven ground the
  "ground plane" is whatever locally-dominant flat patch RANSAC finds under
  and ahead of the robot — it is not a full-terrain elevation estimate (that
  is `elevation_mapping_cupy`'s job).
- **`height_reference_frame` requires a flat, driftless reference.** It is
  only valid while the reference frame's z=0 plane truly is the ground —
  true for `LOCALIZATION_MODE=ground_truth` in Gazebo, false the moment SLAM
  drift or sloped terrain enters the picture (offline video, real hardware,
  or `LOCALIZATION_MODE=slam`). Use `expected_base_height` there instead.
- **Assumes a level body** when using `expected_base_height` — pitch/roll of
  `base_link` shifts the true camera height and isn't corrected for.
- **Scale-only model.** This module tests exactly one hypothesis (pure
  multiplicative scale error). It has no way to distinguish a shift-only or
  mixed affine error from noise — that diagnosis has to come from watching
  `scale` stability and fit RMS together, as described above, or from
  building the quantitative comparison harness mentioned in `CLAUDE.md`'s
  Current Target section.

---

## Source layout

| File | Role |
|---|---|
| `ground_plane_estimation/plane_fit.py` | RANSAC + SVD refinement, pure numpy, ROS-free |
| `ground_plane_estimation/ground_plane_node.py` | ROS node: TF lookup, candidate gating, publishing |
| `launch/ground_plane.launch.py` | Spawns one node per `input_topics` entry, keyed by topic name |
| `test/test_plane_fit.py` | Host-runnable: recovery under clutter, scale-proportionality, steep-plane rejection, degenerate-input rejection |
| `test/test_node_smoke.py` | ROS-environment: synthetic floor cloud → `/ground_plane/distance` within 10 s |
| `README.md` | Module-local quick reference (parameters, run commands) |
