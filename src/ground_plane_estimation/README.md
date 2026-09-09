# ground_plane_estimation

Detects the ground plane in a depth point cloud (RANSAC), measures the
robot's perpendicular distance to it, and publishes the result — the kinetic
probe for the monocular depth **scale hypothesis**: if metric depth error is a
per-frame scale factor `s` (`depth_pred ≈ s · depth_true`), the measured
camera height above ground is exactly `s ×` the true camera height, so
`/ground_plane/scale` is a direct per-frame estimate of `s` that needs no
ground-truth depth image.

Why the *camera* height is the pure observable: the rigid TF into the base
frame only rotates the camera-relative cloud (rotation preserves distances),
so a scaled cloud yields a scaled camera-to-plane distance. The base_link
distance additionally mixes in the un-scaled TF translation; it is published
for plotting and sanity checks, not for estimating `s`.

## Pipeline

```
PointCloud2 ─► TF to base frame ─► low-z candidate gate ─► RANSAC plane
                                                              │
   /ground_plane/distance        sensor_msgs/Range   base_link ⊥ plane
   /ground_plane/camera_height   sensor_msgs/Range   camera origin ⊥ plane
   /ground_plane/scale           std_msgs/Float32    camera_height / expected
   /ground_plane/marker          visualization_msgs/Marker (RViz slab)
```

One node per input topic. The launch file takes a comma-separated
`input_topics` list and namespaces each estimator as
`<output_namespace>/<key>/*` (key from the topic, e.g.
`/depth_compare/dav2_vkitti/points` → `dav2_vkitti`), so every depth backend
of the comparison pipeline gets its own scale channel on identical frames.

## Scale reference (what "expected" means)

| Parameter | Behaviour |
|---|---|
| `expected_base_height` > 0 | Static: nominal standing height of `base_frame` above ground; expected camera height = that + camera z offset in the base frame (assumes a level body). |
| `height_reference_frame` (e.g. `odom`) | Per-frame TF: expected camera height = z of the cloud frame origin in that frame. Correct whenever the reference frame's z=0 plane is the ground — true in Gazebo with `LOCALIZATION_MODE=ground_truth`. |
| neither | `/scale` is not published; distances still are. |

## Run

Gazebo, ground-truth depth (sanity: scale should sit at 1.0):

```bash
docker compose up -d --build ground_plane_estimation
```

Against the depth comparison channels (one estimator per model):

```bash
GROUND_PLANE_INPUT_TOPICS=/depth_compare/gt/points,/depth_compare/dav2_vkitti/points,/depth_compare/dav2_hypersim/points,/depth_compare/metric3d_s/points \
docker compose --env-file depth-comparison.env \
  -f docker-compose-depth-comparison.yaml up -d --build ground_plane_estimation
```

Plot the per-model scales:

```bash
ros2 topic echo /ground_plane/dav2_vkitti/scale
rqt_plot /ground_plane/gt/scale/data /ground_plane/dav2_vkitti/scale/data \
         /ground_plane/dav2_hypersim/scale/data /ground_plane/metric3d_s/scale/data
```

## Key parameters

| Parameter | Default | Meaning |
|---|---|---|
| `input_topic` | `/depth_camera/points` | PointCloud2 to fit (node-level; launch takes `input_topics`) |
| `base_frame` | `base_link` | Distance reference; TF `base_frame ← cloud frame` must exist (identity if the cloud is already in it) |
| `min_range` / `max_range` | 0.2 / 8.0 | Candidate gate in the cloud frame, cloud units |
| `candidate_z_percentile` | 60 | Keep only the lowest fraction by base-frame z before RANSAC (percentile-based, so it survives unknown scale) |
| `ransac_threshold` | 0.03 m | Inlier distance |
| `max_ground_angle_deg` | 35 | Reject planes steeper than this vs. base +z |
| `min_inliers` | 100 | Reject unsupported fits |
| `target_fps` | 5.0 | Upper bound; extra clouds are dropped |

## Tests

`test/test_plane_fit.py` runs on the host (pure numpy): recovery under
clutter, the distance-scales-with-cloud-scale property, steep-plane and
degenerate-input rejection. `test/test_node_smoke.py` needs a ROS
environment: synthetic floor cloud in → Range ≈ 0.18 m out within 10 s.

## Caveats

- On rough terrain the "ground plane" is a local fit; the distance is to the
  dominant plane under/ahead of the robot, not terrain-following clearance.
- A per-frame *shift* (affine depth error, common for relative-depth models)
  bends the plane rather than moving it uniformly — watch the fit RMS/inlier
  logs; a scale-only error keeps the plane flat.
- `height_reference_frame` is only valid while the reference ground really is
  that frame's z=0 plane (flat benchmark worlds, not slopes).
