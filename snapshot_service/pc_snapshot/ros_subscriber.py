"""ROS 2 subscriber for the point-cloud topic.

Runs rclpy in a dedicated background thread so the HTTP API is independent of
ROS callbacks. On every valid message it decodes XYZ(+intensity) and updates
the SnapshotStore with an immutable snapshot. Rendering never touches this
thread: the API captures a read-only reference and renders elsewhere.

This module imports rclpy lazily (inside RosCloudListener.start) so the rest
of the package (decoder, renderer, API tests) works without a sourced ROS
environment.
"""

from __future__ import annotations

import logging
import threading
from typing import Optional

from .decoder import PointCloudDecodeError, decode_pointcloud2
from .store import SnapshotStore

log = logging.getLogger(__name__)


class RosCloudListener:
    """Background rclpy subscriber feeding a SnapshotStore."""

    def __init__(self, topic: str, store: SnapshotStore,
                 domain_id: Optional[str] = None):
        self.topic = topic
        self.store = store
        self.domain_id = domain_id
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._started = False
        self._start_error: Optional[str] = None

    # ------------------------------------------------------------------
    def start(self) -> None:
        import os
        if self.domain_id is not None:
            os.environ["ROS_DOMAIN_ID"] = str(self.domain_id)
        try:
            import rclpy  # noqa: F401
        except Exception as exc:
            self._start_error = f"rclpy unavailable: {exc}"
            log.error(self._start_error)
            raise RuntimeError(self._start_error)
        self._thread = threading.Thread(target=self._run, name="ros-cloud-listener",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    @property
    def alive(self) -> bool:
        return self._started and self._thread is not None and self._thread.is_alive()

    @property
    def start_error(self) -> Optional[str]:
        return self._start_error

    # ------------------------------------------------------------------
    def _run(self) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2

        rclpy.init()
        node = None
        try:
            node = Node("pc_snapshot_subscriber")

            def _on_msg(msg: "PointCloud2") -> None:
                try:
                    decoded = decode_pointcloud2(msg)
                except PointCloudDecodeError as exc:
                    self.store.record_error(f"decode failed on {self.topic}: {exc}")
                    log.warning("decode failed: %s", exc)
                    return
                if decoded.points.shape[0] == 0:
                    self.store.record_error(f"empty cloud on {self.topic}")
                    return
                stamp = 0.0
                try:
                    stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
                except Exception:
                    pass
                frame_id = ""
                try:
                    frame_id = msg.header.frame_id
                except Exception:
                    pass
                self.store.update(decoded, frame_id=frame_id, ros_stamp=stamp,
                                  topic=self.topic)

            # Best-effort sensor-data QoS works with both reliable (default)
            # and best-effort publishers, including `ros2 bag play` output.
            sub = node.create_subscription(PointCloud2, self.topic, _on_msg,
                                           qos_profile_sensor_data)
            log.info("subscribed to %s (depth=%s)",
                     self.topic, qos_profile_sensor_data.depth)
            self._started = True
            while not self._stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception as exc:
            self._start_error = str(exc)
            log.error("ROS listener stopped with error: %s", exc)
            self.store.record_error(f"ROS listener error: {exc}")
        finally:
            self._started = False
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
            log.info("ROS listener exited")
