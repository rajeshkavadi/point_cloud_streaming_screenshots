#!/usr/bin/env bash
# Start both the snapshot service and (optionally) bag playback in the
# background, logging to .run/.
#
# Usage:
#   ./start_all.sh                      # service only
#   ./start_all.sh /path/to/bag.db3     # service + looping bag playback
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$PROJ_DIR/.run"
mkdir -p "$RUN_DIR"

# Clean up any previous run first so repeated starts can't stack orphaned
# processes that hold port 8085 and answer health checks with stale data.
"$SCRIPT_DIR/stop_all.sh" >/dev/null 2>&1 || true

BAG="${1:-}"
TOPIC="${PCSNAP_TOPIC:-/ouster/points}"
PORT="${PCSNAP_PORT:-8085}"
CLIP="${PCSNAP_CLIP_RANGE_M:-60}"

echo "Starting snapshot service (topic=$TOPIC, port=$PORT, clip=${CLIP}m)..."
"$SCRIPT_DIR/run_service.sh" "$TOPIC" "$PORT" "$CLIP" \
  > "$RUN_DIR/service.log" 2>&1 &
echo $! > "$RUN_DIR/service.pid"
echo "  pid $(cat "$RUN_DIR/service.pid"), log $RUN_DIR/service.log"

if [ -n "$BAG" ]; then
  echo "Starting bag playback: $BAG"
  "$SCRIPT_DIR/run_bag.sh" "$BAG" > "$RUN_DIR/bag.log" 2>&1 &
  echo $! > "$RUN_DIR/bag.pid"
  echo "  pid $(cat "$RUN_DIR/bag.pid"), log $RUN_DIR/bag.log"
fi

sleep 2
echo
echo "Health check:"
curl -s "http://127.0.0.1:$PORT/api/health" | python3 -m json.tool || \
  echo "service not up yet; check $RUN_DIR/service.log"
echo
echo "Open http://127.0.0.1:$PORT/ in a browser. Stop with scripts/stop_all.sh"
