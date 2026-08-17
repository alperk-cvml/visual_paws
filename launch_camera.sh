#!/bin/bash
#
docker-compose -f docker-compose-camera-pipeline.yaml --env-file offline-video.env up -d  && ./tmux-attach.sh
docker-compose -f docker-compose-camera-pipeline.yaml down

