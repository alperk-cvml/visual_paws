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
  depth_image_proc
  elevation_mapping
  quadruped_gait_controller
  quadruped_test_controller
)

# ── helpers ───────────────────────────────────────────────────────────────────
die() { echo "ERROR: $*" >&2; exit 1; }

command -v tmux  &>/dev/null || die "tmux not found – install it first"
command -v docker &>/dev/null || die "docker not found"

# ── create sessions ───────────────────────────────────────────────────────────
for CNAME in "${CONTAINERS[@]}"; do
  # kill existing session so we start clean
  tmux has-session -t "$CNAME" 2>/dev/null && tmux kill-session -t "$CNAME"

  if [[ "$CNAME" == "quadruped_test_controller" ]]; then
    # keyboard controller needs a real tty — use `docker attach` with stdin
    tmux new-session -d -s "$CNAME" \
      "docker attach \"$CNAME\" 2>&1 || { echo '[container not running]'; read; }"
  else
    tmux new-session -d -s "$CNAME" \
      "docker attach --no-stdin \"$CNAME\" 2>&1 || { echo '[container not running]'; read; }"
  fi
done

FIRST="${CONTAINERS[0]}"

# ── attach or leave in background ─────────────────────────────────────────────
if [[ "${1}" == "--detach" ]]; then
  echo "tmux sessions created in background:"
  for CNAME in "${CONTAINERS[@]}"; do
    echo "  tmux attach -t $CNAME"
  done
else
  echo "tmux sessions created. Attaching to $FIRST..."
  echo "(Use your tmux prefix + d to detach and see other sessions)"
  tmux attach -t "$FIRST"
fi
