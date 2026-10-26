#!/usr/bin/env bash
# Play a ROS 2 bag (.db3 or bag directory) in a loop for snapshot testing.
#
# Usage:
#   ./run_bag.sh /path/to/Outdoor_cut1_dataset_ros2.db3 [rate]
#
# The bare .db3 file has no metadata.yaml; rosbag2 in Humble can play a
# sqlite3 file directly by passing the file path.
set -eo pipefail

BAG="${1:?usage: run_bag.sh <bag-path.db3|bag-dir> [rate]}"
RATE="${2:-1.0}"

# ROS setup scripts reference unset variables; tolerate that while sourcing.
set +u
source /opt/ros/humble/setup.bash
set -u

echo "Playing bag: $BAG at rate ${RATE}x (looping, Ctrl-C to stop)"
exec ros2 bag play "$BAG" --loop -r "$RATE"
