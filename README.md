Confirmed — exactly one service, one bag, one tunnel (the earlier "2" was just my own shell command matching the text pattern). The stack is clean and healthy.
The bug is fixed
My script had a set -eo pipefail bug: the URL-wait loop ran grep against the tunnel log before Cloudflare had written anything to it. grep returned non-zero (no match yet), and set -e aborted the whole script right after [3/3] — so it never reached the print block. The background processes had already launched (that's why things were running), but the link was never shown.
Fix: the grep now has || true so an empty log just retries instead of killing the script.
Your launch command + output
cd /home/rajesh/point_cloud_streaming_screenshots/snapshot_service
source /opt/ros/humble/setup.bash
./scripts/start_remote.sh /home/rajesh/proc_dem_creation/OutdoorRoad_dataset_ros2/OutdoorRoad_dataset_ros2.db3
It now prints (verified, exit code 0):
==========================================================
  OPEN IN BROWSER (remote OK):
  https://sleeping-knife-lawyers-eating.trycloudflare.com

  API token (paste in the UI's token box): lidar-snap-2026
  Local-only URL: http://127.0.0.1:8085/
==========================================================
…followed by a health check and Stop everything with: ./scripts/stop_all.sh.
Current live state (verified through the public URL)
- Open this on your friend's PC: https://sleeping-knife-lawyers-eating.trycloudflare.com
- Token: lidar-snap-2026
- Public health: OK · ROS connected · 131,072 pts · not stale
- Exactly 1 service + 1 bag + 1 tunnel (no orphans)
Reminder: the quick-tunnel URL is new again (it changed from the last one). Always use the fresh URL that the script prints. ./scripts/stop_all.sh stops all three.
Still open if you want them:
- Wire up FAST-LIO2 (configs already exist) for registered/motion-compensated clouds instead of the raw sweep — I'd test before claiming it works.
- Named tunnel for a permanent URL that never changes.
