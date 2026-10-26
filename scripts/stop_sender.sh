#!/usr/bin/env bash
# Stop the relay and bag player started by start_sender.sh.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$ROOT/.run"

for name in bag relay; do
  pidfile="$RUN/$name.pid"
  if [[ -f "$pidfile" ]]; then
    pid="$(cat "$pidfile")"
    if kill -0 "$pid" 2>/dev/null; then
      # kill the whole process group (ros2 bag play spawns children)
      pkill -TERM -P "$pid" 2>/dev/null || true
      kill -TERM "$pid" 2>/dev/null || true
      echo "[stop_sender] stopped $name (pid $pid)"
    else
      echo "[stop_sender] $name not running"
    fi
    rm -f "$pidfile"
  fi
done
