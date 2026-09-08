#!/usr/bin/env bash
# tmux-attach.sh — attach to all running visual_paws containers via tmux
#
# Creates a distinct tmux session for each module.
# Each session runs `docker attach` so you see the container's live stdout/stderr.
# The keyboard_controller session additionally gets stdin forwarded (tty preserved).
#
# Usage:
#   ./tmux-attach.sh          # create sessions & attach your terminal to the first one
#   ./tmux-attach.sh --detach # create sessions but leave them in background
#
# Prerequisites: tmux, docker

CONTAINERS=(
  quadruped_gazebo
  rviz
  depth_image_proc
 # elevation_mapping
  elevation_mapping_cupy
  monocular_depth
  depth_comparison
  localization
  nav2_mppi_navigation
  quadruped_gait_controller
  quadruped_leg_odometry
  quadruped_test_controller
  video_camera_sim
)

# ── helpers ───────────────────────────────────────────────────────────────────
die() { echo "ERROR: $*" >&2; exit 1; }

command -v tmux  &>/dev/null || die "tmux not found – install it first"
command -v docker &>/dev/null || die "docker not found"

# ── create sessions ───────────────────────────────────────────────────────────
for CNAME in "${CONTAINERS[@]}"; do
  # kill existing session so we start clean
  tmux has-session -t "$CNAME" 2>/dev/null && tmux kill-session -t "$CNAME"

  if [[ "$CNAME" == "quadruped_test_controller" || "$CNAME" == "video_camera_sim" ]]; then
    # interactive controllers need a real tty — use `docker attach` with stdin
    tmux new-session -d -s "$CNAME" \
      "docker attach \"$CNAME\" 2>&1 || { echo '[container not running]'; read; }"
  else
    tmux new-session -d -s "$CNAME" \
      "docker attach --no-stdin \"$CNAME\" 2>&1 || { echo '[container not running]'; read; }"
  fi
done

# Create a session to launch rqt_image_view inside monocular_depth container
CNAME="monocular_depth_rqt"
tmux has-session -t "$CNAME" 2>/dev/null && tmux kill-session -t "$CNAME"
tmux new-session -d -s "$CNAME" \
  "docker exec -it monocular_depth bash -lc 'source /work/ws/install/setup.bash && ros2 run rqt_image_view rqt_image_view' 2>&1 || { echo '[container not running or rqt_image_view failed]'; read; }"

FIRST="${CONTAINERS[0]}"

# ── attach or leave in background ─────────────────────────────────────────────
if [[ "${1}" == "--detach" ]]; then
  echo "tmux sessions created in background:"
  for CNAME in "${CONTAINERS[@]}"; do
    echo "  tmux attach -t $CNAME"
  done
  echo "  tmux attach -t monocular_depth_rqt"
else
  echo "tmux sessions created. Attaching to $FIRST..."
  echo "(Use your tmux prefix + d to detach and see other sessions)"
  tmux attach -t "$FIRST"
fi
