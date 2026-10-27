"""Live stream tests: StreamManager frame publication + HTTP endpoint.

Uses a stub render_fn so no Open3D/Vulkan context is needed; the real
renderer path is covered indirectly by test_renderer.py.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pytest
from fastapi.testclient import TestClient

from pc_snapshot.api import create_app
from pc_snapshot.config import Config
from pc_snapshot.jobs import RenderCoordinator, SnapshotRegistry
from pc_snapshot.store import SnapshotStore
from pc_snapshot.stream import (BOUNDARY, StreamManager, jpeg_bytes,
                                mjpeg_generator, multipart_part)
from tests.fixtures import make_scene_cloud
from pc_snapshot.decoder import decode_pointcloud2


def _stub_render(snap, views, cfg):
    """Deterministic fake renderer: solid 64x48 RGB per requested view."""
    arr = np.full((48, 64, 3), 42, dtype=np.uint8)
    return {v: arr.copy() for v in views}


@pytest.fixture()
def env(tmp_path):
    cfg = Config()
    cfg.api_token = ""
    cfg.image_width = 320
    cfg.image_height = 180
    cfg.stream_idle_stop_sec = 0.5      # fast idle-stop for tests
    cfg.stream_fps = 20.0
    cfg.validate()
    store = SnapshotStore(topic="/test/points", stale_after_sec=5.0)
    registry = SnapshotRegistry(tmp_path / "snaps", max_entries=5,
                                retention_sec=300.0)
    coordinator = RenderCoordinator(cfg, registry)
    stream = StreamManager(cfg, store, render_fn=_stub_render)
    app = create_app(cfg, store, coordinator, listener=None, stream=stream)
    client = TestClient(app)
    return client, store, cfg, stream


def _feed(store: SnapshotStore, n: int = 3000) -> None:
    dec = decode_pointcloud2(make_scene_cloud(n))
    store.update(dec, frame_id="camera_init", ros_stamp=time.time(),
                 topic="/test/points")


# ---------------------------------------------------------------------------
# Unit: multipart framing + manager
# ---------------------------------------------------------------------------

def test_multipart_part_format():
    part = multipart_part(b"\xff\xd8FAKEJPEG\xff\xd9")
    assert part.startswith(b"--frame\r\nContent-Type: image/jpeg\r\n")
    assert b"Content-Length: 12\r\n" in part
    assert part.endswith(b"\xff\xd8FAKEJPEG\xff\xd9\r\n")


def test_jpeg_bytes_is_valid_jpeg():
    arr = np.zeros((16, 16, 3), dtype=np.uint8)
    data = jpeg_bytes(arr, quality=50)
    assert data[:2] == b"\xff\xd8"      # SOI marker
    assert data[-2:] == b"\xff\xd9"     # EOI marker


def test_manager_publishes_frames_for_active_view(env):
    client, store, cfg, stream = env
    _feed(store)
    assert stream.add_client("front") is True
    try:
        # The loop first publishes a "starting_renderer" placeholder, then
        # real frames — wait for the first OK frame.
        seq = jpeg = meta = None
        deadline = time.time() + 5.0
        last = 0
        while time.time() < deadline:
            frame = stream.wait_for_frame("front", last, timeout=2.0)
            if frame is None:
                continue
            last = frame[0]
            if frame[2].get("status") == "ok":
                seq, jpeg, meta = frame
                break
        assert seq is not None, "no OK frame within 5s"
        assert seq > 0
        assert jpeg[:2] == b"\xff\xd8"
        assert meta["points"] > 0
        # wait_for_frame only ever returns frames NEWER than last_seq.
        again = stream.wait_for_frame("front", seq, timeout=0.3)
        assert again is None or again[0] > seq
    finally:
        stream.remove_client("front")


def test_manager_placeholder_when_no_data(env):
    client, store, cfg, stream = env
    # No messages fed -> loop publishes placeholders (starting_renderer
    # first, then waiting_for_data). Wait for the data-status one.
    stream.add_client("top")
    try:
        found = None
        deadline = time.time() + 5.0
        last = 0
        while time.time() < deadline:
            frame = stream.wait_for_frame("top", last, timeout=2.0)
            if frame is None:
                continue
            last = frame[0]
            if frame[2].get("status") == "waiting_for_data":
                found = frame
                break
        assert found is not None, "no waiting_for_data placeholder within 5s"
        _, jpeg, meta = found
        assert jpeg[:2] == b"\xff\xd8"
        assert meta["status"] == "waiting_for_data"
    finally:
        stream.remove_client("top")


def test_manager_idle_stops_render_loop(env):
    client, store, cfg, stream = env
    _feed(store)
    stream.add_client("front")
    assert stream.status()["render_loop_alive"] is True
    stream.remove_client("front")
    # stream_idle_stop_sec = 0.5 -> loop should exit shortly after.
    deadline = time.time() + 3.0
    while stream.status()["render_loop_alive"] and time.time() < deadline:
        time.sleep(0.05)
    assert stream.status()["render_loop_alive"] is False
    assert stream.status()["clients"] == 0


def test_manager_second_client_does_not_spawn_second_thread(env):
    client, store, cfg, stream = env
    _feed(store)
    assert stream.add_client("top") is True
    assert stream.add_client("left") is False    # thread already running
    try:
        assert stream.status()["clients"] == 2
        assert set(stream.status()["active_views"]) == {"top", "left"}
    finally:
        stream.remove_client("top")
        stream.remove_client("left")


# ---------------------------------------------------------------------------
# HTTP endpoint
# ---------------------------------------------------------------------------

def test_stream_requires_token_when_configured(env):
    client, store, cfg, stream = env
    cfg.api_token = "secret"
    r = client.get("/api/stream")
    assert r.status_code == 401
    r = client.get("/api/stream", params={"token": "wrong"})
    assert r.status_code == 401
    cfg.api_token = ""      # restore for other assertions in this fixture


def test_stream_rejects_unknown_view(env):
    client, store, cfg, stream = env
    r = client.get("/api/stream", params={"view": "diagonal"})
    assert r.status_code == 400
    assert "diagonal" in r.json()["detail"]


def test_stream_disabled_returns_404(env):
    client, store, cfg, stream = env
    cfg.stream_enabled = False
    r = client.get("/api/stream")
    assert r.status_code == 404
    cfg.stream_enabled = True


def test_stream_endpoint_yields_multipart_headers():
    """HTTP contract via TestClient: status, content-type, client cleanup.

    starlette's TestClient buffers whole responses (no incremental reads of
    infinite streams), so body framing is asserted directly against
    mjpeg_generator in test below; here we only prove the endpoint wires
    things up correctly.
    """
    cfg = Config()
    cfg.api_token = ""
    cfg.validate()
    store = SnapshotStore(topic="/test/points", stale_after_sec=5.0)
    registry = SnapshotRegistry(Path("/tmp/pc_snap_test") / "snaps",
                                max_entries=5, retention_sec=300.0)
    coordinator = RenderCoordinator(cfg, registry)

    class StreamDone(Exception):
        pass

    class EndAfterOneManager(StreamManager):
        """No threads: one canned frame, then the response ends."""

        def __init__(self):
            super().__init__(cfg, store)
            self.removed_views = []

        def add_client(self, view):
            with self._lock:
                self._clients += 1
                self._views[view] = 1
            return False

        def remove_client(self, view):
            self.removed_views.append(view)
            with self._lock:
                self._clients = max(0, self._clients - 1)
                self._views.pop(view, None)

        def wait_for_frame(self, view, last_seq, timeout=3.0):
            if last_seq > 0:
                raise StreamDone()
            arr = np.full((48, 64, 3), 7, dtype=np.uint8)
            return last_seq + 1, jpeg_bytes(arr, 50), {"status": "ok"}

    mgr = EndAfterOneManager()
    app = create_app(cfg, store, coordinator, listener=None, stream=mgr)
    client = TestClient(app, raise_server_exceptions=False)

    r = client.get("/api/stream", params={"view": "left"})
    assert r.status_code == 200
    assert "multipart/x-mixed-replace" in r.headers["content-type"]
    assert f"boundary={BOUNDARY}" in r.headers["content-type"]
    # Client cleanup ran when the response ended.
    assert mgr.removed_views == ["left"]
    assert mgr.client_count == 0


def test_mjpeg_generator_frames_and_cleanup():
    """Body framing: each yielded part is a complete multipart JPEG section,
    and closing the generator removes the client (server-side disconnect)."""
    cfg = Config()
    cfg.validate()
    store = SnapshotStore(topic="/test/points", stale_after_sec=5.0)

    class CannedManager(StreamManager):
        def __init__(self, n=3):
            super().__init__(cfg, store)
            self.n = n
            self.removed_views = []

        def add_client(self, view):
            with self._lock:
                self._clients += 1
            return False

        def remove_client(self, view):
            self.removed_views.append(view)
            with self._lock:
                self._clients = max(0, self._clients - 1)

        def wait_for_frame(self, view, last_seq, timeout=3.0):
            if last_seq >= self.n:
                time.sleep(0.05)      # simulate blocking, never a frame
                return None
            arr = np.full((48, 64, 3), last_seq, dtype=np.uint8)
            return last_seq + 1, jpeg_bytes(arr, 50), {"status": "ok"}

    mgr = CannedManager(n=3)
    gen = mjpeg_generator(mgr, "left")
    parts = []
    for part in gen:
        parts.append(part)
        if len(parts) == 3:
            break
    gen.close()      # what the server does on client disconnect

    assert len(parts) == 3
    for i, part in enumerate(parts, start=1):
        header, payload_rest = part.split(b"\r\n\r\n", 1)
        assert header.startswith(b"--frame\r\n")
        assert b"Content-Type: image/jpeg" in header
        assert f"Content-Length:".encode() in header
        payload = payload_rest[:-2]        # strip trailing \r\n
        assert payload[:2] == b"\xff\xd8"  # JPEG SOI
        assert payload[-2:] == b"\xff\xd9"  # JPEG EOI
        assert int(header.split(b"Content-Length: ")[1].split(b"\r\n")[0]) \
            == len(payload)
    # The generator's finally removed the client.
    assert mgr.removed_views == ["left"]
    assert mgr.client_count == 0


def test_mjpeg_generator_sends_placeholder_while_waiting():
    """When no frames exist yet the placeholder JPEG is streamed (heartbeat)."""
    cfg = Config()
    cfg.validate()
    store = SnapshotStore(topic="/test/points", stale_after_sec=5.0)
    mgr = StreamManager(cfg, store)
    gen = mjpeg_generator(mgr, "top", wait_timeout=0.05, heartbeat_sec=0.0)
    part = next(gen)
    gen.close()
    header, payload_rest = part.split(b"\r\n\r\n", 1)
    payload = payload_rest[:-2]
    assert payload == mgr.placeholder_jpeg
    assert payload[:2] == b"\xff\xd8"
    assert mgr.client_count == 0          # closed -> removed


def test_health_includes_stream_status(env):
    client, store, cfg, stream = env
    h = client.get("/api/health").json()
    assert "stream" in h
    s = h["stream"]
    assert s["enabled"] is True
    assert s["clients"] == 0
    assert s["resolution"] == [cfg.stream_width, cfg.stream_height]
    assert s["target_fps"] == cfg.stream_fps
