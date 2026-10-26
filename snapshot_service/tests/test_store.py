"""Tests for snapshot store behaviour: latest-message, staleness, consistency."""

import threading
import time

from pc_snapshot.decoder import decode_pointcloud2
from pc_snapshot.store import SnapshotStore
from tests.fixtures import make_xyz_cloud


def _store_update(store: SnapshotStore, n: int, frame: str = "camera_init"):
    dec = decode_pointcloud2(make_xyz_cloud(n))
    return store.update(dec, frame_id=frame, ros_stamp=1.0 + n, topic="/test")


def test_latest_message_wins():
    store = SnapshotStore(topic="/test")
    _store_update(store, 100)
    snap = store.capture()
    assert snap is not None and snap.point_count == 100
    _store_update(store, 200)
    snap2 = store.capture()
    assert snap2.point_count == 200 and snap2.seq == 2
    assert snap2.frame_id == "camera_init"


def test_capture_none_before_any_message():
    store = SnapshotStore(topic="/test")
    assert store.capture() is None
    h = store.health()
    assert h["message_received"] is False
    assert h["latest_message_age_sec"] is None


def test_empty_message_rejected():
    store = SnapshotStore(topic="/test")
    dec = decode_pointcloud2(make_xyz_cloud(0))
    assert store.update(dec, frame_id="f", ros_stamp=0.0) is False
    assert store.capture() is None
    assert store.health()["messages_rejected"] == 1


def test_capture_is_immutable_reference():
    """Capture returns the frozen snapshot; arrays are read-only."""
    store = SnapshotStore(topic="/test")
    _store_update(store, 100)
    snap = store.capture()
    import numpy as np
    with np.errstate(all="raise"):
        try:
            snap.points[0] = 999.0
            raised = False
        except ValueError:
            raised = True
    assert raised, "snapshot points must be read-only"


def test_staleness():
    store = SnapshotStore(topic="/test", stale_after_sec=0.05)
    _store_update(store, 10)
    assert store.age_seconds() < 0.05
    assert store.health()["stale"] is False
    time.sleep(0.08)
    assert store.age_seconds() > 0.05
    assert store.health()["stale"] is True


def test_snapshot_consistency_across_captures():
    """The same captured snapshot must keep the same points for all four
    renders even if new messages arrive concurrently."""
    store = SnapshotStore(topic="/test")
    _store_update(store, 500)
    snap = store.capture()
    # Newer messages arrive after capture...
    _store_update(store, 1234)
    _store_update(store, 777)
    # ...but the captured snapshot is unchanged.
    assert snap.point_count == 500
    assert snap.seq == 1


def test_thread_safety():
    store = SnapshotStore(topic="/test")
    errors = []

    def writer(k):
        try:
            for i in range(200):
                _store_update(store, 50 + (k + i) % 10)
        except Exception as exc:  # pragma: no cover
            errors.append(exc)

    def reader():
        try:
            for _ in range(400):
                store.capture()
                store.health()
        except Exception as exc:  # pragma: no cover
            errors.append(exc)

    threads = [threading.Thread(target=writer, args=(k,)) for k in range(3)]
    threads += [threading.Thread(target=reader) for _ in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert not errors
    assert store.stats.messages_received == 600
