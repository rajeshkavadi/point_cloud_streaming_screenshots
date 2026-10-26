#!/usr/bin/env bash
# One command: start snapshot service + bag playback + Cloudflare tunnel,
# then print the clickable public URL and API token.
#
# Usage:
#   ./scripts/start_remote.sh /path/to/bag.db3 [topic]
#
# Env overrides:
#   PCSNAP_API_TOKEN   API token (default: lidar-snap-2026)
#   PCSNAP_PORT        HTTP port  (default: 8085)
#   PCSNAP_CLIP_RANGE_M radial clip in meters (default: 60)
#
# Stop everything with: ./scripts/stop_all.sh
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$PROJ_DIR/.run"
mkdir -p "$RUN_DIR"

BAG="${1:?usage: start_remote.sh /path/to/bag.db3 [topic]}"
TOPIC="${2:-/ouster/points}"
PORT="${PCSNAP_PORT:-8085}"
CLIP="${PCSNAP_CLIP_RANGE_M:-60}"
TOKEN="${PCSNAP_API_TOKEN:-lidar-snap-2026}"

if [ ! -e "$BAG" ]; then
  echo "ERROR: bag path not found: $BAG" >&2
  exit 1
fi

# Clean up any previous run first (service, bag, tunnel) so we never stack
# orphans or leave a stale process holding the port.
"$SCRIPT_DIR/stop_all.sh" >/dev/null 2>&1 || true

echo "[1/3] Starting snapshot service (topic=$TOPIC, port=$PORT, clip=${CLIP}m)..."
PCSNAP_API_TOKEN="$TOKEN" "$SCRIPT_DIR/run_service.sh" "$TOPIC" "$PORT" "$CLIP" \
  > "$RUN_DIR/service.log" 2>&1 &
echo $! > "$RUN_DIR/service.pid"

echo "[2/3] Starting bag playback: $BAG"
"$SCRIPT_DIR/run_bag.sh" "$BAG" > "$RUN_DIR/bag.log" 2>&1 &
echo $! > "$RUN_DIR/bag.pid"

echo "[3/3] Starting Cloudflare tunnel..."
CF="$(command -v cloudflared || echo "$HOME/.local/bin/cloudflared")"
if [ ! -x "$CF" ]; then
  echo "ERROR: cloudflared not found. Install it first (see README 7.7)." >&2
  exit 1
fi
nohup "$CF" tunnel --url "http://localhost:$PORT" > "$RUN_DIR/tunnel.log" 2>&1 &
echo $! > "$RUN_DIR/tunnel.pid"

# Wait for the public URL to appear in the tunnel log.
# NOTE: must not fail the script if the log is empty on early iterations
# (grep returns non-zero, which under `set -eo pipefail` would abort).
URL=""
for _ in $(seq 1 30); do
  URL="$(grep -oE 'https://[a-z0-9-]+\.trycloudflare\.com' "$RUN_DIR/tunnel.log" 2>/dev/null | head -1 || true)"
  if [ -n "$URL" ]; then break; fi
  sleep 1
done

# Wait briefly for the service to accept connections + bag discovery.
sleep 5

echo
if [ -n "$URL" ]; then
  echo "=========================================================="
  echo "  OPEN IN BROWSER (remote OK):"
  echo "  $URL"
  echo
  echo "  API token (paste in the UI's token box): $TOKEN"
  echo "  Local-only URL: http://127.0.0.1:$PORT/"
  echo "=========================================================="
else
  echo "WARNING: tunnel URL not found yet. Check $RUN_DIR/tunnel.log"
fi

echo
echo "Health check:"
curl -s "http://127.0.0.1:$PORT/api/health" \
  -H "X-API-Token: $TOKEN" | python3 -m json.tool \
  || echo "service not up yet; check $RUN_DIR/service.log"
echo
echo "Stop everything with:  ./scripts/stop_all.sh"
echo "Logs:  $RUN_DIR/service.log  $RUN_DIR/bag.log  $RUN_DIR/tunnel.log"
