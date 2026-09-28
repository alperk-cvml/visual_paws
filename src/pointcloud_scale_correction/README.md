# pointcloud_scale_correction

Applies the monocular depth scale estimate from `ground_plane_estimation` to
the point cloud itself, republishing a metric-corrected copy for downstream
consumers (elevation mapping, planning).

```
/depth/points ─────────────────┐
                               ▼
/ground_plane/scale ──► CorrectionModel ──► /depth/points_corrected
(/ground_plane/shift,          │                    ├─ /applied_scale
 future estimator) ────────────┘                    └─ /applied_shift (affine)
```

## Error models

The correction math is a pluggable `CorrectionModel`
(`correction_models.py`, pure numpy, host-testable). The current working
hypothesis is a scalar error; the suspected fallback is affine. Both are
implemented — switching is a single parameter, no rewiring:

| Model | Error assumed | Correction | Frame requirement |
|---|---|---|---|
| `scalar` (default) | `depth_pred = s·depth_true` | `p / s` | any frame (uniform scale commutes with rotation) |
| `affine` | `depth_pred = s·depth_true + b` | `p · ((z−b)/s)/z` per ray | **camera optical frame** (z = depth); a depth shift does not commute with rotation |

With `shift ≡ 0` the affine model reduces exactly to the scalar one. No
estimator publishes `b` yet — the ground-plane probe alone cannot separate
`s` from `b`; when one exists (e.g. the IMU + leg-odometry motion probe,
Alternative 2), point `shift_topics` at it and set `model:=affine`.

To add another error model (e.g. depth-dependent scale), subclass
`CorrectionModel` and register it in `MODELS`; the node and launch file pick
it up by name.

## Behaviour

- **Header preserved verbatim** — the corrected cloud keeps the input's
  `frame_id` and stamp (repo TF rule: never restamp or reframe a cloud).
  Output is an unorganized XYZ32 cloud; non-XYZ fields are dropped.
- **Estimates are smoothed** inside the model (`smoothing`: `none` / `ema` /
  `median`) and gated (`min_scale`/`max_scale`) so one bad plane fit doesn't
  warp a cloud.
- **Stale/no estimate** (`estimate_timeout`, default 5 s) → `stale_policy`:
  `hold` the last good estimate (default), `passthrough` uncorrected, or
  `drop`. Before the first estimate, `hold` drops rather than silently
  passing wrong-scale clouds.
- The applied parameters are republished as `<output>/applied_scale` (and
  `/applied_shift` for affine) for rqt_plot next to the raw
  `/ground_plane/scale`.

## Parameters (per node)

| Parameter | Default | Meaning |
|---|---|---|
| `input_topic` | `/depth_camera/points` | PointCloud2 to correct |
| `output_topic` | `''` | Empty derives `<input_topic>_corrected` |
| `scale_topic` | `/ground_plane/scale` | Float32 scale estimate source |
| `shift_topic` | `''` | Optional Float32 shift estimate (affine only) |
| `model` | `scalar` | `scalar` or `affine` |
| `smoothing` | `ema` | `none`, `ema`, or `median` |
| `ema_alpha` | `0.2` | EMA weight of a new observation |
| `median_window` | `5` | Rolling median window length |
| `min_scale` / `max_scale` | `0.05` / `20.0` | Observation sanity gate |
| `estimate_timeout` | `5.0` | Seconds before an estimate is stale; 0 disables |
| `stale_policy` | `hold` | `hold`, `passthrough`, or `drop` |

The launch file (`scale_correction.launch.py`) takes comma-separated
`input_topics` / `scale_topics` / `shift_topics` / `output_topics` and spawns
one node per input, mirroring `ground_plane.launch.py` — a single scale topic
is shared across all inputs, or give one per input to pair with the depth
comparison channels.

## Run

Against the depth-scale-probe profile (Metric3D + ground-truth-referenced
probe already running):

```bash
docker compose up -d --build pointcloud_scale_correction
ros2 topic echo /depth_camera/points_corrected/applied_scale
```

Compare in RViz: raw cloud vs. `<input>_corrected` should differ by exactly
the plotted scale.

## Tests

```bash
pytest src/pointcloud_scale_correction/test/test_correction_models.py  # host, numpy only
pytest src/pointcloud_scale_correction/test/                           # + smoke test inside a ROS env
```
