#!/usr/bin/env bash
# Start the snapshot HTTP service (foreground).
#
# Usage:
#   ./run_service.sh [topic] [port] [clip_range_m]
#
# Defaults come from the config: /ouster/points, port 8085.
# clip_range_m trims sparse far returns so the dense scene fills the frame
# (0 = disabled). 60 is a good default for a raw spinning LiDAR.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"

# ROS setup scripts reference unset variables; tolerate that while sourcing.
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$PROJ_DIR"

TOPIC="${1:-}"
PORT="${2:-}"
CLIP="${3:-}"

ARGS=()
if [ -n "$TOPIC" ]; then ARGS+=(--topic "$TOPIC"); fi
if [ -n "$PORT" ]; then ARGS+=(--port "$PORT"); fi
if [ -n "$CLIP" ]; then ARGS+=(--clip-range "$CLIP"); fi

exec python3 -m pc_snapshot "${ARGS[@]}"
