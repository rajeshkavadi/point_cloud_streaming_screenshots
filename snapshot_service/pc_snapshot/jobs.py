"""Bounded snapshot-job execution and on-disk snapshot registry.

- A non-blocking semaphore bounds simultaneous render jobs; a second request
  while one is rendering returns HTTP 409 instead of piling up.
- Rendered PNGs are written to a dedicated temp directory under a generated
  snapshot id (never a client-supplied path).
- A registry with a max-entry cap plus a TTL sweeper thread provides bounded,
  automatically cleaned-up storage.
"""

from __future__ import annotations

import io
import json
import logging
import re
import shutil
import threading
import time
import uuid
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional

from .config import Config
from .renderer import VIEW_NAMES, RenderError, images_to_png_bytes, render_snapshot
from .store import PointCloudSnapshot

log = logging.getLogger(__name__)

SNAPSHOT_ID_RE = re.compile(r"^[0-9a-f]{32}$")


class JobBusyError(Exception):
    """Another snapshot render is already running."""


class SnapshotNotFound(Exception):
    pass


@dataclass
class SnapshotArtifact:
    snapshot_id: str
    created_walltime: float
    meta: dict
    files: Dict[str, Path] = field(default_factory=dict)

    @property
    def dir(self) -> Path:
        return self.files["top"].parent


class SnapshotRegistry:
    """Bounded, TTL-cleaned storage of rendered snapshots."""

    def __init__(self, base_dir: Path, max_entries: int, retention_sec: float):
        self.base_dir = base_dir
        self.max_entries = max(1, max_entries)
        self.retention_sec = max(10.0, retention_sec)
        self._lock = threading.Lock()
        self._entries: Dict[str, SnapshotArtifact] = {}
        self._stop = threading.Event()
        base_dir.mkdir(parents=True, exist_ok=True)
        self._sweeper = threading.Thread(target=self._sweep_loop,
                                         name="snapshot-ttl-sweeper", daemon=True)
        self._sweeper.start()

    def new_id(self) -> str:
        return uuid.uuid4().hex

    def create_dir(self, snapshot_id: str) -> Path:
        if not SNAPSHOT_ID_RE.match(snapshot_id):
            raise ValueError("invalid snapshot id")
        d = self.base_dir / snapshot_id
        d.mkdir(parents=True, exist_ok=False)
        return d

    def put(self, artifact: SnapshotArtifact) -> None:
        with self._lock:
            self._entries[artifact.snapshot_id] = artifact
            # Enforce the entry cap by evicting the oldest snapshots.
            while len(self._entries) > self.max_entries:
                oldest_id = min(self._entries,
                                key=lambda k: self._entries[k].created_walltime)
                self._evict(oldest_id)

    def get(self, snapshot_id: str) -> SnapshotArtifact:
        if not SNAPSHOT_ID_RE.match(snapshot_id):
            raise SnapshotNotFound("invalid snapshot id format")
        with self._lock:
            art = self._entries.get(snapshot_id)
        if art is None or not art.files:
            raise SnapshotNotFound(f"snapshot {snapshot_id} not found or expired")
        return art

    def _evict(self, snapshot_id: str) -> None:
        art = self._entries.pop(snapshot_id, None)
        if art is not None:
            shutil.rmtree(art.dir, ignore_errors=True)

    def _sweep_loop(self) -> None:
        while not self._stop.wait(30.0):
            try:
                cutoff = time.time() - self.retention_sec
                with self._lock:
                    stale = [sid for sid, art in self._entries.items()
                             if art.created_walltime < cutoff]
                    for sid in stale:
                        self._evict(sid)
                if stale:
                    log.info("swept %d expired snapshots", len(stale))
            except Exception:  # pragma: no cover - defensive
                log.exception("snapshot sweeper error")

    def shutdown(self) -> None:
        self._stop.set()
        shutil.rmtree(self.base_dir, ignore_errors=True)


class RenderCoordinator:
    """Runs render jobs with a bounded number of concurrent jobs."""

    def __init__(self, cfg: Config, registry: SnapshotRegistry):
        self.cfg = cfg
        self.registry = registry
        self._sem = threading.Semaphore(max(1, cfg.max_concurrent_renders))

    def run(self, snap: PointCloudSnapshot) -> SnapshotArtifact:
        """Render one snapshot and store the PNGs. Blocking; call from a
        worker thread. Raises RenderError or JobBusyError."""
        if not self._sem.acquire(blocking=False):
            raise JobBusyError("another snapshot render is already in progress")
        t0 = time.time()
        try:
            images, rendered_count = render_snapshot(snap, self.cfg)
            pngs = images_to_png_bytes(images)
            snapshot_id = self.registry.new_id()
            out_dir = self.registry.create_dir(snapshot_id)
            files: Dict[str, Path] = {}
            for view in VIEW_NAMES:
                p = out_dir / f"{view}.png"
                p.write_bytes(pngs[view])
                files[view] = p
            meta = {
                "snapshot_id": snapshot_id,
                "capture_timestamp": time.time(),
                "ros_message_stamp": snap.ros_stamp,
                "frame_id": snap.frame_id,
                "topic": snap.topic,
                "original_point_count": snap.total_points_in_message,
                "valid_point_count": snap.point_count,
                "rendered_point_count": rendered_count,
                "message_age_at_capture_sec":
                    round(time.time() - snap.received_walltime, 3),
                "views": list(VIEW_NAMES),
                "image_size": [self.cfg.image_width, self.cfg.image_height],
                "up_axis": self.cfg.up_axis,
                "color_mode": self.cfg.color_mode,
                "render_backend": self.cfg.renderer,
                "render_seconds": round(time.time() - t0, 3),
            }
            (out_dir / "metadata.json").write_text(
                json.dumps(meta, indent=2), encoding="utf-8")
            artifact = SnapshotArtifact(snapshot_id=snapshot_id,
                                        created_walltime=time.time(),
                                        meta=meta, files=files)
            self.registry.put(artifact)
            log.info("snapshot %s rendered in %.2fs (%s points)",
                     snapshot_id, meta["render_seconds"],
                     meta["rendered_point_count"])
            return artifact
        finally:
            self._sem.release()


def build_zip(artifact: SnapshotArtifact) -> bytes:
    """Return a ZIP containing the four PNGs plus metadata.json."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        for view, path in artifact.files.items():
            zf.write(path, arcname=f"{artifact.snapshot_id}/{view}.png")
        zf.writestr(f"{artifact.snapshot_id}/metadata.json",
                    json.dumps(artifact.meta, indent=2))
    return buf.getvalue()
