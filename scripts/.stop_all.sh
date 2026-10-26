#!/usr/bin/env bash
# internal: kill background demo processes
pkill -f 'pc_[r]elay'
pkill -f '[s]ynthetic_cloud'
pkill -f 'ros2 bag [p]lay'
sleep 1
exit 0
