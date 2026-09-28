# depth_comparison

Side-by-side comparison of monocular metric depth models. One node runs every
configured backend on the **same** RGB frame and publishes one metric depth
image per model; the launch file fans each one out to its own point cloud.

```
/front_camera/image_raw ──┬─→ relay (Gazebo depth sensor) ─→ /depth_compare/gt/{image_raw,camera_info}       ─→ /depth_compare/gt/points
                          ├─→ DA-V2 metric (vkitti)       ─→ /depth_compare/dav2_vkitti/{...}                ─→ /depth_compare/dav2_vkitti/points
                          ├─→ DA-V2 metric (hypersim)     ─→ /depth_compare/dav2_hypersim/{...}              ─→ /depth_compare/dav2_hypersim/points
                          └─→ Metric3D V2 (vit_small)     ─→ /depth_compare/metric3d_s/{...}                 ─→ /depth_compare/metric3d_s/points
```

Every depth image carries the **source image header verbatim**, so all four
point clouds land in the same optical frame at the same instant and overlay
directly in RViz.

## Why one process

Rate-limiting four separate depth nodes lets them drift onto different source
frames, and comparing depth maps of different moments measures nothing. A single
process guarantees identical pixels in, identical timestamps out.

## Backends

| Key | Type | Scale comes from |
|---|---|---|
| `gt` | `relay` | Gazebo depth sensor — the reference channel |
| `dav2_vkitti` | `depth_anything_v2` | VKITTI training domain (driving, ~1.5 m camera height), 80 m |
| `dav2_hypersim` | `depth_anything_v2` | Hypersim training domain (indoor rooms), 20 m |
| `metric3d_s` | `metric3d_v2` | The live camera intrinsics (canonical 1000 px focal) |

The split that matters: DA-V2 metric checkpoints are **intrinsics-agnostic** —
scale is baked in from the training domain. Metric3D is
**intrinsics-conditioned** — it reads focal length from `CameraInfo`. That is
the hypothesis under test in `DEPTH_SCALE_DIAGNOSIS.md`.

## Run

```bash
# Gazebo (default)
docker compose up -d --build depth_comparison

# Offline video — no ground-truth sensor, so drop the `gt` channel
COMPARE_INPUT_IMAGE_TOPIC=/camera/image_raw \
COMPARE_INPUT_CAMERA_INFO_TOPIC=/camera/camera_info \
COMPARE_BACKENDS=dav2_vkitti,dav2_hypersim,metric3d_s \
  docker compose --env-file offline-video.env up -d --build depth_comparison
```

The node logs a distribution table every `log_stats_every` frames. Real output
from the three inference channels on an RTX 2060, driven by a **synthetic test
frame** — the timings are representative, the depths are not:

```
depth comparison over 20 frames
  backend                 p05   median      p95    avg ms
  dav2_vkitti            3.96     4.23     4.72     341.3
  dav2_hypersim          1.52     1.61     1.75     295.6
  metric3d_s             0.79     0.83     0.85    1167.4
```

A model whose percentiles sit well above the reference is overestimating
near-field geometry — the failure that blows up the accumulated elevation map.

### Throughput

All three models run on every accepted frame, so the per-frame cost is their
sum: roughly **1.8 s/frame (~0.55 Hz)** on an RTX 2060. Metric3D dominates
because it always infers at its 616×1064 canonical resolution regardless of
input size. `target_fps` is an upper bound, not a promise — the node simply
drops whatever it cannot keep up with. Run two models at a time if you want a
responsive loop.

## Metric3D and the mmcv shim

Metric3D imports **mmcv 1.x** on its inference path (`mono/utils/comm.py` does
`from mmcv.utils import collect_env` unconditionally). Neither real mmcv works
here: 1.x fails to install on Python 3.12, and 2.x removed the `mmcv.utils`
namespace entirely, moving its contents into mmengine.

[`compat/mmcv/`](../../compat/mmcv/) supplies the four symbols Metric3D actually
imports — `collect_env`, `Config`, `DictAction`, `get_git_hash` — re-exported
from mmengine, and raises `ImportError` on anything else so an upstream change
fails loudly. The Metric3D checkout in the torch hub cache is never modified.

The shim lives at the repository root because `monocular_depth` needs it too —
it now runs Metric3D as the production depth source. That is also why both
images build with the repository root as their Docker context.

Verified behaviour: doubling `fx` doubles predicted depth (measured ratio 1.99
against an expected 2.00), confirming Metric3D recovers metric scale from the
camera intrinsics rather than from a training prior.

## Adding a model

1. Implement `DepthBackend` in `depth_comparison/backends/`.
2. Register the class in `backends/__init__.py` under `BACKEND_TYPES`.
3. Add a channel to `config/backends.yaml` and its key to `backends`.

The node declares the backend's parameters from `param_defaults()` and the
launch file spawns the matching point cloud. Nothing else changes.

## Gazebo ground-truth channel

`src/quadruped_gazebo/urdf/quadruped.urdf.xacro` puts both sensors on
`camera_link` at an explicit zero pose with matching FOV and clip planes, and
both publish with `gz_frame_id` `camera_link_optical`. Verified through the SDF
conversion, they resolve to an identical pose, so the ground-truth cloud
overlays the inferred ones with no parallax and no rotation.

Two differences remain between the `gt` channel and the model channels, both
intentional:

- **Resolution** — the depth sensor is 160×120 against the RGB camera's
  640×480. The FOV matches, so the intrinsics stay consistent and the cloud is
  correct, just 16× sparser. Raise it to 640×480 if you want a per-pixel
  quantitative comparison rather than a visual one.
- **Timing** — the relay republishes at the sensor's 10 Hz, independent of the
  node's RGB-driven inference rate. Fine for RViz inspection; a per-pixel metric
  would need the two synchronised on timestamp first.
