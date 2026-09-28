# VisualPaws SAR Simulation Benchmark

## Purpose

VisualPaws needs a reproducible Gazebo simulation environment for testing camera-first quadruped autonomy in outdoor search-and-rescue-like terrain.

The simulator is primarily intended to validate:

* terrain representation
* depth → point cloud → elevation map behavior
* traversability estimation
* localization / map accumulation
* local planning
* quadruped locomotion over geometrically difficult terrain
* system-level failure and recovery behavior

Photorealism is not the primary objective. Geometry, reproducibility, and measurable difficulty are more important.

The benchmark will be developed incrementally.

## V0 — Geometry Benchmark

### Goal

Create a deterministic, procedurally generated outdoor terrain benchmark that stresses elevation mapping, traversability estimation, planning, and locomotion without introducing perception or mission-level confounding factors.

V0 is a geometry and integration benchmark. It may be used to compare how the pipeline responds to synthetic RGB and ground-truth depth, but it must not be presented as validation of monocular depth performance on real outdoor imagery.

### Scope

V0 includes:

* uneven natural terrain
* slopes of varying steepness
* rough ground
* deterministic rock fields
* concrete and rubble obstacles
* a ditch or depression
* a narrow bridge
* a constrained passage
* flat, collision-free spawn and finish pads
* easy, moderate, and hard traversability levels

The initial benchmark should contain three parallel lanes, one per difficulty level. Each lane should expose the robot to the same classes of feature with increasingly severe dimensions. Fixed lane positions and feature envelopes make runs comparable; the seed changes only stochastic details such as natural roughness, rock placement, and rubble orientation.

Difficulty labels initially describe relative geometric severity, not guaranteed robot capability. They must be calibrated using observed mapping, planning, and locomotion results before they are treated as pass/fail boundaries.

### Scenario Definition

The source of truth will be a versioned YAML scenario, planned at:

```text
src/quadruped_gazebo/config/benchmarks/v0_geometry.yaml
```

The scenario must define:

* `schema_version` and scenario name
* default random seed
* world dimensions and heightfield resolution
* surface friction
* robot spawn pose and finish regions
* deterministic terrain-generation parameters
* named lanes and difficulty labels
* every feature's stable ID, type, pose, dimensions, and parameters
* reserved clear areas and allowed feature bounds

All distances use metres and all angles use radians, following REP-103. Coordinates are expressed in the Gazebo world coordinate system; generated ROS frames must continue to use the repository's canonical REP-105 frame names.

An indicative schema is:

```yaml
schema_version: 1
name: v0_geometry
random_seed: 42

world:
  size: [18.0, 12.0]
  height_samples: [257, 257]
  base_elevation: 0.0
  friction: 0.9
  static_model_clearance: 0.003
  sky_color: [0.45, 0.68, 0.92, 1.0]

spawn:
  position: [-7.5, 0.0, 0.35]
  yaw: 0.0

terrain:
  base_roughness_amplitude: 0.008
  correlation_length: 0.5

lanes:
  - id: easy
    center_y: 3.0
    difficulty: easy
    features: []
  - id: moderate
    center_y: 0.0
    difficulty: moderate
    features: []
  - id: hard
    center_y: -3.0
    difficulty: hard
    features: []
```

The final scenario must contain fully specified features; the empty lists above illustrate structure only.

### Initial Difficulty Calibration

The following values are starting points for simulation testing. They should remain configuration values rather than constants embedded in generator code.

| Feature | Easy | Moderate | Hard |
|---|---:|---:|---:|
| Slope | 0.087 rad | 0.175 rad | 0.262 rad |
| Roughness amplitude | 0.005 m | 0.012 m | 0.025 m |
| Rock height | 0.01–0.02 m | 0.02–0.04 m | 0.04–0.07 m |
| Ditch width | 0.25 m | 0.40 m | 0.60 m |
| Ditch depth | 0.08 m | 0.12 m | 0.18 m |
| Bridge width | 0.70 m | 0.50 m | 0.35 m |
| Passage width | 0.70 m | 0.50 m | 0.35 m |

Some moderate and hard features may intentionally exceed the fixed gait controller's locomotion capability. A feature can still be valuable when the correct result is an impassable traversability classification or a planned route around it.

### Generation Contract

The generator must produce the same output for the same scenario, seed, generator version, and container environment. The random-number algorithm and generator schema version must be explicit so an implementation change cannot silently alter an existing benchmark.

The seed may affect:

* natural heightfield roughness
* rock positions and dimensions within fixed bounds
* rubble positions and orientations within fixed bounds

The seed must not affect:

* world dimensions
* lane positions
* spawn and finish pads
* feature types or named feature envelopes
* camera, robot, physics, or TF configuration

The generator should create continuous terrain as one indexed mesh shared by the Gazebo collision and visual elements, and discrete obstacles as named static SDF models. Sharing the mesh is required because native heightmap rendering and heightfield collision can diverge between Gazebo backends. Generated files are build artifacts and must not be committed as the primary benchmark definition.

### Generated Artifacts

A generated run should contain:

```text
world.sdf
terrain.png
terrain.obj
elevation.npy
manifest.json
```

`terrain.obj` is the geometry Gazebo uses for both rendering and collision. `terrain.png` remains a portable visualization of the height field; it is not loaded independently by Gazebo and therefore cannot introduce a visual-to-collision offset.

The manifest provides ground truth for inspection and future quantitative evaluation. It must record:

* scenario schema and generator versions
* seed and source YAML checksum
* generated world, terrain image, terrain mesh, and elevation checksums
* terrain resolution, origin, size, and elevation range
* spawn pose and finish regions
* exact feature poses, dimensions, parameters, and bounding boxes
* lane and difficulty assignments

`elevation.npy` stores the numeric terrain surface before image quantization. Later benchmark versions may consume it for elevation-map error metrics without changing the V0 scenario contract.

### Launch Interface

V0 uses a dedicated environment profile and Compose file so its settings do not leak into the default Gazebo or depth-comparison profiles:

```bash
docker compose --env-file simulation-benchmark.env \
  -f docker-compose-simulation-benchmark.yaml up -d --build
```

The scenario YAML is the source of truth for the default random seed. The environment profile selects the scenario, may override its seed on demand, and defines simulation time, localization mode, and the generated-output directory. The defaults are:

```dotenv
SIM_BENCHMARK_SCENARIO=v0_geometry
SIM_BENCHMARK_SEED=config
USE_SIM_TIME=true
LOCALIZATION_MODE=ground_truth
```

`SIM_BENCHMARK_SEED=config` uses `random_seed: 42` from `v0_geometry.yaml`. To produce a different deterministic world without editing the scenario, set an explicit integer override:

```bash
SIM_BENCHMARK_SEED=43 docker compose \
  --env-file simulation-benchmark.env \
  -f docker-compose-simulation-benchmark.yaml up -d --build
```

Gazebo ground truth owns `odom→base_link` in this profile, with one static `map→odom` publisher. The benchmark must not introduce additional publishers for either TF edge. Images, depth images, camera information, and point clouds must preserve their source timestamps and canonical camera optical frame.

### V0 Validation Workflow

The first validation pass is visual and integration-focused:

1. Generate and validate the scenario artifacts.
2. Start Gazebo with the generated world and ground-truth localization.
3. Confirm the robot spawns stably on the clear pad.
4. Inspect RGB, ground-truth depth, point cloud, elevation map, and traversability outputs in RViz.
5. Drive or plan through each difficulty lane.
6. Record observed mapping, classification, planning, and locomotion failures against the feature IDs in `manifest.json`.
7. Repeat the same seed to confirm reproducibility, then use additional fixed seeds to check sensitivity to stochastic geometry.

Quantitative scoring is not part of V0. The generated elevation and feature manifest merely preserve the information needed to add quantitative comparison without redesigning the scenarios.

### Acceptance Criteria

V0 is complete when:

* the scenario YAML validates before generation
* duplicate feature IDs, invalid dimensions, overlaps with reserved pads, and out-of-bounds features fail with actionable errors
* identical inputs produce byte-identical generated artifacts within the supported container image
* changing the seed changes stochastic details while preserving fixed feature envelopes
* the generated SDF passes Gazebo validation
* the robot spawns on a flat, collision-free pad
* `/clock`, canonical TF, RGB, ground-truth depth, camera information, and ground-truth odometry are available
* each lane and named feature can be inspected in Gazebo and RViz
* ground-truth depth produces an elevation map containing the expected named features
* existing default Gazebo and depth-comparison launch profiles continue to work
* generated artifacts are excluded from version control

### Explicitly Out of Scope

Do not add in V0:

* victims or SAR mission logic
* benchmark scoring or leaderboards
* smoke, dust, or communication degradation
* sophisticated vegetation
* photorealistic PBR work
* dynamic actors
* camera defects or complex perception degradation
* automated claims about real-world monocular-depth accuracy

V0 exists to establish a clean, measurable, and reproducible geometric baseline.

## V1 — Perception Benchmark

Add visual complexity that challenges RGB perception and monocular depth:

* realistic materials
* vegetation
* shadows
* difficult lighting
* texture variation
* visually ambiguous terrain

## V2 — SAR Mission Benchmark

Add mission-level semantics:

* victims
* search regions
* collapsed structures
* mission objectives
* benchmark scoring
* ground-truth victim / hazard information

## V3 — Adversarial Benchmark

Add degraded operating conditions:

* smoke and dust
* camera degradation
* communication loss
* dynamic people / obstacles
* extreme illumination
* localization degradation
* system-level fault scenarios

## Architecture Principle

Scenario definition and Gazebo world representation should be separated.

Preferred architecture:

scenario YAML
↓
procedural world generator
↓
generated terrain / SDF
↓
Gazebo

The same scenario and seed must produce the same environment.

Generated files should not become the primary source of truth. Parameters and generation code should be.

## Benchmark Philosophy

Prefer controlled and measurable geometric difficulty over decorative complexity.

A visually simple obstacle with known dimensions is more useful than a realistic asset whose geometry is difficult to reason about.

Every important terrain feature should eventually be describable numerically so that perception and traversability outputs can be compared against ground truth.
