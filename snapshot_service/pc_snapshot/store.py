"""Latest-message holder and immutable snapshot capture.

The subscriber thread calls update() with each decoded cloud. The HTTP
handler calls capture() to obtain an immutable snapshot of exactly one
message. Rendering operates only on the snapshot, never on the live buffer,
so the subscriber is never blocked by rendering.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .decoder import DecodedCloud


@dataclass(frozen=True)
class PointCloudSnapshot:
    """One immutable captured point cloud message."""

    points: np.ndarray            # (N, 3) float32, read-only
    intensity: Optional[np.ndarray]  # (N,) float32 read-only or None
    frame_id: str
    ros_stamp: float              # ROS header stamp in seconds (0.0 if unknown)
    received_walltime: float      # host clock at receive time (time.time())
    topic: str
    total_points_in_message: int  # points before non-finite filtering
    seq: int                      # monotonically increasing message counter

    @property
    def point_count(self) -> int:
        return int(self.points.shape[0])


@dataclass
class StoreStats:
    messages_received: int = 0
    messages_rejected: int = 0  # decodes that produced no valid points
    captures: int = 0
    last_error: Optional[str] = None


class SnapshotStore:
    """Thread-safe holder for the latest valid point cloud."""

    def __init__(self, topic: str = "", stale_after_sec: float = 5.0):
        self._lock = threading.Lock()
        self._latest: Optional[PointCloudSnapshot] = None
        self._seq = 0
        self.topic = topic
        self.stale_after_sec = stale_after_sec
        self.stats = StoreStats()

    # -- subscriber side ---------------------------------------------------
    def update(self, decoded: DecodedCloud, frame_id: str, ros_stamp: float,
               topic: Optional[str] = None) -> bool:
        """Store the latest decoded cloud. Returns False if it was empty."""
        with self._lock:
            self.stats.messages_received += 1
            if decoded.points.shape[0] == 0:
                self.stats.messages_rejected += 1
                return False
            self._seq += 1
            self._latest = PointCloudSnapshot(
                points=decoded.points,
                intensity=decoded.intensity,
                frame_id=frame_id or "",
                ros_stamp=float(ros_stamp),
                received_walltime=time.time(),
                topic=topic or self.topic,
                total_points_in_message=decoded.total_points,
                seq=self._seq,
            )
            return True

    def record_error(self, message: str) -> None:
        with self._lock:
            self.stats.last_error = message
            self.stats.messages_rejected += 1

    # -- request side ------------------------------------------------------
    def capture(self) -> Optional[PointCloudSnapshot]:
        """Return an immutable reference to the latest valid message.

        The snapshot arrays are read-only; rendering never mutates the live
        subscriber buffer.
        """
        with self._lock:
            if self._latest is None:
                return None
            self.stats.captures += 1
            return self._latest

    # -- diagnostics -------------------------------------------------------
    def health(self) -> dict:
        with self._lock:
            latest = self._latest
            age = None if latest is None else time.time() - latest.received_walltime
            return {
                "topic": self.topic,
                "message_received": latest is not None,
                "latest_message_age_sec": None if age is None else round(age, 3),
                "stale": bool(age is not None and age > self.stale_after_sec),
                "stale_after_sec": self.stale_after_sec,
                "latest_frame_id": None if latest is None else latest.frame_id,
                "latest_ros_stamp": None if latest is None else latest.ros_stamp,
                "latest_point_count": None if latest is None else latest.point_count,
                "messages_received": self.stats.messages_received,
                "messages_rejected": self.stats.messages_rejected,
                "captures": self.stats.captures,
                "last_error": self.stats.last_error,
            }

    def age_seconds(self) -> Optional[float]:
        with self._lock:
            if self._latest is None:
                return None
            return time.time() - self._latest.received_walltime
