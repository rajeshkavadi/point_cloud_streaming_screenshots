#!/usr/bin/env bash
# Publish a synthetic spinning cloud (no dataset needed) — quick viewer test.
# Usage: scripts/run_synthetic.sh [TOPIC] [RATE] [POINTS]
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
exec python3 "$ROOT/relay/synthetic_cloud.py" \
  --topic "${1:-/synthetic/points}" --rate "${2:-10}" --points "${3:-20000}"
