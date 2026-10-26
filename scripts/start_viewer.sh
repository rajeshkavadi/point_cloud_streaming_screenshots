#!/usr/bin/env bash
# Serve the web viewer on http://localhost:8000 (Mode A: WSL -> Windows browser
# reaches it via WSL2's automatic localhost forwarding).
# Env: VIEWER_PORT (8000)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$ROOT/.run"
mkdir -p "$RUN"
PORT="${VIEWER_PORT:-8000}"

if [[ -f "$RUN/viewer.pid" ]] && kill -0 "$(cat "$RUN/viewer.pid")" 2>/dev/null; then
  echo "viewer already running (pid $(cat "$RUN/viewer.pid")) — http://localhost:$PORT"
  exit 0
fi

nohup python3 -m http.server "$PORT" --bind 0.0.0.0 --directory "$ROOT/web" \
  > "$RUN/viewer.log" 2>&1 &
echo $! > "$RUN/viewer.pid"
echo "[start_viewer] serving $ROOT/web on port $PORT (pid $(cat "$RUN/viewer.pid"))"
echo "Open in the browser:  http://localhost:$PORT"
echo "WSL eth0 IP (for Tailscale/portproxy): $(hostname -I | awk '{print $1}')"
