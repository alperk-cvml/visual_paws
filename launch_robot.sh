#!/bin/bash
docker-compose down && docker-compose up -d && ./tmux-attach.sh
