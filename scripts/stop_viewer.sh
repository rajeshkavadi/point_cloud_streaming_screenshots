#!/usr/bin/env bash
# Stop the web viewer.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pidfile="$ROOT/.run/viewer.pid"
if [[ -f "$pidfile" ]]; then
  pid="$(cat "$pidfile")"
  kill "$pid" 2>/dev/null && echo "[stop_viewer] stopped (pid $pid)" || echo "[stop_viewer] not running"
  rm -f "$pidfile"
fi
