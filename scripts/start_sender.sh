#!/usr/bin/env bash
# Start the point-cloud sender: binary WebSocket relay (+ optional bag replay).
#
# Usage:
#   scripts/start_sender.sh [BAG_PATH] [TOPIC]
#   BAG_PATH: directory with metadata.yaml OR a .db3 file. Omit to skip bag
#             playback (e.g. when using relay/synthetic_cloud.py or a live node).
#   TOPIC:    PointCloud2 topic (default /ouster/points for the Outdoor_cut1 bag)
#
# Env overrides: RELAY_PORT (9091), RELAY_RATE (10), MAX_POINTS (50000),
#                CLIP_RANGE (120), RELAY_HOST (0.0.0.0), BAG_LOOP (1)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$ROOT/.run"
mkdir -p "$RUN"

source /opt/ros/humble/setup.bash

BAG="${1:-}"
TOPIC="${2:-/ouster/points}"
RELAY_HOST="${RELAY_HOST:-0.0.0.0}"
RELAY_PORT="${RELAY_PORT:-9091}"
RELAY_RATE="${RELAY_RATE:-10}"
MAX_POINTS="${MAX_POINTS:-50000}"
CLIP_RANGE="${CLIP_RANGE:-120}"

if [[ -f "$RUN/relay.pid" ]] && kill -0 "$(cat "$RUN/relay.pid")" 2>/dev/null; then
  echo "relay already running (pid $(cat "$RUN/relay.pid"))"
else
  echo "[start_sender] starting relay on ws://${RELAY_HOST}:${RELAY_PORT} topic=${TOPIC}"
  nohup python3 "$ROOT/relay/pc_relay.py" \
    --topic "$TOPIC" --host "$RELAY_HOST" --port "$RELAY_PORT" \
    --rate "$RELAY_RATE" --max-points "$MAX_POINTS" --clip-range "$CLIP_RANGE" \
    > "$RUN/relay.log" 2>&1 &
  echo $! > "$RUN/relay.pid"
  echo "[start_sender] relay pid $(cat "$RUN/relay.pid"), log $RUN/relay.log"
fi

if [[ -n "$BAG" ]]; then
  if [[ -f "$RUN/bag.pid" ]] && kill -0 "$(cat "$RUN/bag.pid")" 2>/dev/null; then
    echo "bag player already running (pid $(cat "$RUN/bag.pid"))"
  else
    LOOP_FLAG=""
    [[ "${BAG_LOOP:-1}" == "1" ]] && LOOP_FLAG="--loop"
    echo "[start_sender] playing bag $BAG (topic $TOPIC) $LOOP_FLAG"
    # shellcheck disable=SC2086
    nohup ros2 bag play "$BAG" --topics "$TOPIC" $LOOP_FLAG \
      > "$RUN/bag.log" 2>&1 &
    echo $! > "$RUN/bag.pid"
    echo "[start_sender] bag player pid $(cat "$RUN/bag.pid"), log $RUN/bag.log"
  fi
fi

sleep 1
echo "[start_sender] recent relay log:"
tail -n 5 "$RUN/relay.log" || true
