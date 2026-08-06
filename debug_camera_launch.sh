#!/bin/bash
docker-compose -f docker-compose-camera-pipeline.yaml down

docker compose --env-file offline-video.env \
  -f docker-compose-camera-pipeline.yaml \
  up -d --force-recreate --remove-orphans

./tmux-attach.sh
