# Standalone RViz

This service runs RViz independently of Gazebo. Both the root simulation stack
and `docker-compose-camera-pipeline.yaml` include it.

The default config is `config/visual_paws.rviz`, with `map` as its fixed frame.
It includes the GridMap and Nav2 RViz plugins needed by this repository.

Run only RViz from the repository root:

```bash
docker compose up rviz --build
```

Run it with the offline-video clock settings:

```bash
docker compose --env-file offline-video.env \
  -f docker-compose-camera-pipeline.yaml up rviz --build
```

Override the config or clock mode with `RVIZ_CONFIG` and `USE_SIM_TIME`.
