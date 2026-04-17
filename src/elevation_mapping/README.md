# elevation_mapping

Lightweight CPU-based 2.5-D elevation mapping node for ROS 2 Jazzy.

Consumes the point cloud produced by the `dept_image_proc` container and publishes a `grid_map_msgs/GridMap` message that can be visualised in RViz.

## Architecture

```
dept_image_proc  ──▶  /depth_camera/depth_image/points (PointCloud2)
                                 │
                    elevation_mapping (this package)
                                 │
                    /elevation_mapping/elevation_map (GridMap)
```

The node (`simple_elevation_mapping`) looks up the TF transform from the sensor frame to the map frame for every incoming point cloud, projects each point onto the XY plane, and records the **maximum Z value** per grid cell (max-height fusion).

## Package structure

```
elevation_mapping/
├── Dockerfile                          # ROS 2 Jazzy image (apt only, no CUDA)
├── docker-compose.yaml
└── simple_elevation_mapping/           # ROS 2 ament_python package
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    └── simple_elevation_mapping/
        └── elevation_mapping_node.py   # Main ROS 2 node
```

## Quick start

```bash
cd src/elevation_mapping

# Build the Docker image (first time only)
docker compose build

# Run
docker compose up
```

## Configuration

All parameters are set via environment variables in `docker-compose.yaml` or on the command line:

| Variable | Default | Description |
|---|---|---|
| `POINTCLOUD_TOPIC` | `/depth_camera/depth_image/points` | Input point cloud topic |
| `MAP_FRAME_ID` | `odom` | Fixed map frame |
| `SENSOR_FRAME_ID` | `depth_camera_link` | Sensor frame (used for TF lookup) |
| `MAP_LENGTH_X` | `3.0` | Map extent in X (metres) |
| `MAP_LENGTH_Y` | `3.0` | Map extent in Y (metres) |
| `MAP_RESOLUTION` | `0.05` | Cell size (metres) |
| `PUBLISH_RATE` | `2.0` | GridMap publish rate (Hz) |
| `ROS_DOMAIN_ID` | `0` | ROS 2 domain ID |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | RMW middleware |

Example override:
```bash
MAP_RESOLUTION=0.02 MAP_LENGTH_X=5.0 docker compose up
```

## Visualisation

1. Install the RViz plugin on the host (if not already present):
   ```bash
   sudo apt install ros-jazzy-grid-map-rviz-plugin
   ```
2. In RViz, add a **GridMap** display and set the topic to:
   ```
   /elevation_mapping/elevation_map
   ```

## Dependencies (installed via apt inside the image)

- `ros-jazzy-grid-map`, `ros-jazzy-grid-map-ros`, `ros-jazzy-grid-map-pcl`
- `ros-jazzy-pcl-ros`, `ros-jazzy-tf2-ros`, `ros-jazzy-tf2-geometry-msgs`
- `python3-numpy`

> **Note:** This package does **not** use the ANYbotics `elevation_mapping` (ROS 1 / catkin) or `elevation_mapping_cupy` (requires CUDA). It is a self-contained ROS 2 implementation built on top of the `grid_map` ecosystem.
