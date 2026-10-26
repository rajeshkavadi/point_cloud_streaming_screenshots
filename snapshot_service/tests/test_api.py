"""API behaviour tests using FastAPI TestClient (no ROS required)."""

import io
import time
import zipfile

import pytest
from fastapi.testclient import TestClient

from pc_snapshot.api import create_app
from pc_snapshot.config import Config
from pc_snapshot.decoder import decode_pointcloud2
from pc_snapshot.jobs import RenderCoordinator, SnapshotRegistry
from pc_snapshot.renderer import VIEW_NAMES
from pc_snapshot.store import SnapshotStore
from tests.fixtures import make_scene_cloud, make_xyz_cloud


@pytest.fixture()
def env(tmp_path):
    cfg = Config()
    cfg.api_token = ""
    cfg.image_width = 320
    cfg.image_height = 180
    cfg.max_render_points = 50_000
    cfg.stale_after_sec = 5.0
    cfg.validate()
    store = SnapshotStore(topic="/test/points", stale_after_sec=5.0)
    registry = SnapshotRegistry(tmp_path / "snaps", max_entries=5,
                                retention_sec=300.0)
    coordinator = RenderCoordinator(cfg, registry)
    app = create_app(cfg, store, coordinator, listener=None)
    client = TestClient(app)
    return client, store, cfg


def _feed(store: SnapshotStore, n_ground: int, frame: str = "camera_init") -> int:
    """Feed one synthetic scene; returns the decoded point count."""
    dec = decode_pointcloud2(make_scene_cloud(n_ground))
    store.update(dec, frame_id=frame, ros_stamp=time.time(), topic="/test/points")
    return dec.points.shape[0]


def _force_stale(store: SnapshotStore, age_sec: float = 60.0) -> None:
    """Backdate the latest snapshot's receive time (frozen dataclass)."""
    object.__setattr__(store._latest, "received_walltime",
                       time.time() - age_sec)


def test_health_without_messages(env):
    client, store, cfg = env
    r = client.get("/api/health")
    assert r.status_code == 200
    h = r.json()
    assert h["status"] == "ok"
    assert h["ros"]["listener_alive"] is False
    assert h["point_cloud"]["topic"] == "/test/points"
    assert h["point_cloud"]["message_received"] is False
    assert h["point_cloud"]["latest_message_age_sec"] is None


def test_health_with_message(env):
    client, store, cfg = env
    n = _feed(store, 3000)
    h = client.get("/api/health").json()
    assert h["point_cloud"]["message_received"] is True
    assert h["point_cloud"]["latest_frame_id"] == "camera_init"
    assert h["point_cloud"]["latest_point_count"] == n
    assert h["point_cloud"]["stale"] is False
    assert h["point_cloud"]["latest_message_age_sec"] < 5.0


def test_snapshot_rejected_when_no_data(env):
    client, store, cfg = env
    r = client.post("/api/snapshot")
    assert r.status_code == 409
    assert "no point cloud" in r.json()["detail"]


def test_snapshot_rejected_when_stale_reject_mode(env):
    client, store, cfg = env
    cfg.stale_policy = "reject"
    _feed(store, 1000)
    _force_stale(store)
    r = client.post("/api/snapshot")
    assert r.status_code == 409
    assert "stale" in r.json()["detail"]


def test_snapshot_warns_when_stale_warn_mode(env):
    client, store, cfg = env
    _feed(store, 1000)
    _force_stale(store)
    r = client.post("/api/snapshot")
    assert r.status_code == 200
    body = r.json()
    assert any("stale" in w for w in body["warnings"])


def test_snapshot_full_flow(env):
    client, store, cfg = env
    n = _feed(store, 4000)
    r = client.post("/api/snapshot")
    assert r.status_code == 200, r.text
    body = r.json()
    assert set(body["views"]) == set(VIEW_NAMES)
    assert body["frame_id"] == "camera_init"
    assert body["valid_point_count"] == n
    assert body["rendered_point_count"] == n
    assert body["snapshot_id"]
    assert len(body["snapshot_id"]) == 32
    # All four image URLs return valid PNGs.
    for view in VIEW_NAMES:
        img = client.get(body["images"][view])
        assert img.status_code == 200, view
        assert img.headers["content-type"] == "image/png"
        assert img.content[:8] == b"\x89PNG\r\n\x1a\n"
        assert len(img.content) > 1000


def test_snapshot_images_all_same_snapshot(env):
    """URLs embed the same snapshot id -> same capture."""
    client, store, cfg = env
    _feed(store, 2000)
    body = client.post("/api/snapshot").json()
    sid = body["snapshot_id"]
    for view in VIEW_NAMES:
        assert body["images"][view].endswith(f"/api/snapshot/{sid}/{view}.png")


def test_invalid_view_and_id(env):
    client, store, cfg = env
    _feed(store, 1000)
    body = client.post("/api/snapshot").json()
    sid = body["snapshot_id"]
    r = client.get(f"/api/snapshot/{sid}/bottom.png")
    assert r.status_code == 404
    r = client.get(f"/api/snapshot/not-a-real-id/top.png")
    assert r.status_code == 404
    # Path traversal attempt must not escape the registry.
    r = client.get("/api/snapshot/..%2F..%2Fetc%2Fpasswd/top.png")
    assert r.status_code == 404
    # Valid snapshot: image works but zip for an unknown id must 404.
    assert client.get(f"/api/snapshot/{sid}/top.png").status_code == 200
    assert client.get(
        "/api/snapshot/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa/download.zip"
    ).status_code == 404


def test_download_zip(env):
    client, store, cfg = env
    _feed(store, 2000)
    body = client.post("/api/snapshot").json()
    r = client.get(body["download_zip"])
    assert r.status_code == 200
    assert r.headers["content-type"] == "application/zip"
    zf = zipfile.ZipFile(io.BytesIO(r.content))
    names = zf.namelist()
    sid = body["snapshot_id"]
    for view in VIEW_NAMES:
        assert f"{sid}/{view}.png" in names
    assert f"{sid}/metadata.json" in names
    meta = zf.read(f"{sid}/metadata.json").decode()
    assert '"frame_id": "camera_init"' in meta


def test_registry_evicts_oldest(env):
    client, store, cfg = env
    ids = []
    for _ in range(7):  # cap is 5
        _feed(store, 500)
        ids.append(client.post("/api/snapshot").json()["snapshot_id"])
    # Oldest snapshots evicted -> their images 404.
    assert client.get(
        f"/api/snapshot/{ids[0]}/top.png").status_code == 404
    assert client.get(
        f"/api/snapshot/{ids[-1]}/top.png").status_code == 200


def test_api_token_required(tmp_path):
    cfg = Config()
    cfg.api_token = "secret123"
    cfg.image_width = 160
    cfg.image_height = 90
    cfg.validate()
    store = SnapshotStore(topic="/t", stale_after_sec=5.0)
    registry = SnapshotRegistry(tmp_path / "snaps2", 5, 300.0)
    coordinator = RenderCoordinator(cfg, registry)
    app = create_app(cfg, store, coordinator, listener=None)
    client = TestClient(app)
    _feed(store, 1000)
    assert client.post("/api/snapshot").status_code == 401
    assert client.post("/api/snapshot",
                       headers={"X-API-Token": "wrong"}).status_code == 401
    r = client.post("/api/snapshot", headers={"X-API-Token": "secret123"})
    assert r.status_code == 200
    r = client.post("/api/snapshot?token=secret123")
    assert r.status_code == 200


def test_downsample_warning_in_response(env):
    client, store, cfg = env
    cfg.max_render_points = 1_000
    n = _feed(store, 8000)
    body = client.post("/api/snapshot").json()
    assert body["valid_point_count"] == n
    assert body["rendered_point_count"] <= 1000
    assert any("downsampled" in w for w in body["warnings"])


def test_frontend_served(env):
    client, store, cfg = env
    r = client.get("/")
    assert r.status_code == 200
    assert "Remote LiDAR Snapshot Viewer" in r.text
    assert "Fetch Point Cloud" in r.text
