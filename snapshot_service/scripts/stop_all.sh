#!/usr/bin/env bash
# Stop background service/bag processes started by start_all.sh.
# Kills by pattern too, so orphaned processes (from repeated start_all runs)
# are caught even if their PID is no longer in the .pid files.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$(dirname "$SCRIPT_DIR")/.run"

# 1) Kill whatever is in the PID files.
for name in service bag; do
  pidfile="$RUN_DIR/$name.pid"
  if [ -f "$pidfile" ]; then
    pid="$(cat "$pidfile")"
    if kill -0 "$pid" 2>/dev/null; then
      echo "Stopping $name (pid $pid)"
      kill "$pid" 2>/dev/null || true
    fi
    rm -f "$pidfile"
  fi
done

# 2) Kill any orphans by pattern (service + bag player + tunnel).
if pgrep -f "python3 -m pc_snapshot" >/dev/null 2>&1; then
  pkill -f "python3 -m pc_snapshot" 2>/dev/null && echo "Stopped stray snapshot service" || true
fi
if pgrep -f "ros2 bag play" >/dev/null 2>&1; then
  pkill -f "ros2 bag play" 2>/dev/null && echo "Stopped stray bag play" || true
fi
if pgrep -f "cloudflared tunnel" >/dev/null 2>&1; then
  pkill -f "cloudflared tunnel" 2>/dev/null && echo "Stopped Cloudflare tunnel" || true
fi

# Give processes a moment to exit so the port is released.
sleep 1
if ss -tln 2>/dev/null | grep -q ':8085'; then
  echo "Warning: port 8085 still in use; forcing."
  pkill -9 -f "python3 -m pc_snapshot" 2>/dev/null || true
  sleep 1
fi
echo "Done."
