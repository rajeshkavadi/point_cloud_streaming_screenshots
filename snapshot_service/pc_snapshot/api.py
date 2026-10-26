"""FastAPI HTTP application.

Endpoints:
    GET  /api/health
    POST /api/snapshot
    GET  /api/snapshot/{snapshot_id}/{view}.png
    GET  /api/snapshot/{snapshot_id}/download.zip
    GET  /                       (static browser frontend)

The app depends only on a SnapshotStore and a RenderCoordinator, so it can be
unit-tested without ROS or a bag.
"""

from __future__ import annotations

import logging
import platform
import time
from pathlib import Path
from typing import Optional

import anyio
from fastapi import Depends, FastAPI, Header, HTTPException, Query, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

from . import __version__
from .config import Config
from .jobs import (JobBusyError, RenderCoordinator, SnapshotNotFound,
                   build_zip)
from .renderer import RenderError, VIEW_NAMES
from .ros_subscriber import RosCloudListener
from .store import SnapshotStore

log = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).resolve().parent.parent / "static"


def create_app(cfg: Config, store: SnapshotStore,
               coordinator: RenderCoordinator,
               listener: Optional[RosCloudListener] = None) -> FastAPI:
    app = FastAPI(title="Remote LiDAR Snapshot Service", version=__version__)
    app.state.cfg = cfg
    app.state.store = store
    app.state.coordinator = coordinator
    app.state.listener = listener

    app.add_middleware(
        CORSMiddleware,
        allow_origins=cfg.cors_origins,
        allow_methods=["GET", "POST"],
        allow_headers=["*"],
    )

    # ------------------------------------------------------------------
    # Optional token auth (empty token = disabled, intended for a private
    # Tailscale network only).
    # ------------------------------------------------------------------
    def require_token(x_api_token: Optional[str] = Header(default=None),
                      token: Optional[str] = Query(default=None)) -> None:
        if not cfg.api_token:
            return
        if x_api_token == cfg.api_token or token == cfg.api_token:
            return
        raise HTTPException(status_code=401, detail="invalid or missing API token")

    # ------------------------------------------------------------------
    @app.get("/api/health")
    def health() -> dict:
        store_health = store.health()
        listener_alive = listener.alive if listener is not None else False
        return {
            "status": "ok",
            "service": "pc-snapshot",
            "version": __version__,
            "ros": {
                "listener_alive": listener_alive,
                "listener_start_error":
                    listener.start_error if listener is not None else None,
                "connected": listener_alive and store_health["message_received"],
            },
            "point_cloud": store_health,
            "render": {
                "renderer": cfg.renderer,
                "image_size": [cfg.image_width, cfg.image_height],
                "max_render_points": cfg.max_render_points,
                "max_concurrent_renders": cfg.max_concurrent_renders,
            },
            "host": {"platform": platform.platform(),
                     "python": platform.python_version()},
            "time": time.time(),
        }

    @app.post("/api/snapshot")
    async def snapshot(request: Request, _: None = Depends(require_token)):
        age = store.age_seconds()
        if age is None:
            raise HTTPException(
                status_code=409,
                detail=(f"no point cloud has been received yet on topic "
                        f"'{cfg.topic}'. Start bag playback or the sensor "
                        f"driver first."))
        if age > cfg.stale_after_sec:
            msg = (f"latest point cloud is stale: last message {age:.1f}s ago "
                   f"(threshold {cfg.stale_after_sec:.1f}s). Bag playback may "
                   f"have stopped or the topic is idle.")
            if cfg.stale_policy == "reject":
                raise HTTPException(status_code=409, detail=msg)
            log.warning(msg)

        snap = store.capture()
        if snap is None or snap.point_count == 0:
            raise HTTPException(
                status_code=409,
                detail="captured message contains no usable points")

        # Render in a worker thread; the subscriber keeps running.
        try:
            artifact = await anyio.to_thread.run_sync(coordinator.run, snap)
        except JobBusyError as exc:
            raise HTTPException(status_code=409, detail=str(exc))
        except RenderError as exc:
            log.error("render failed: %s", exc)
            raise HTTPException(status_code=500, detail=f"rendering failed: {exc}")
        except Exception as exc:  # unexpected
            log.exception("snapshot job failed")
            raise HTTPException(status_code=500, detail=f"snapshot job failed: {exc}")

        meta = artifact.meta
        base = str(request.base_url).rstrip("/")
        images = {
            view: f"{base}/api/snapshot/{artifact.snapshot_id}/{view}.png"
            for view in VIEW_NAMES
        }
        warnings = []
        if age > cfg.stale_after_sec:
            warnings.append(f"snapshot rendered from a stale message ({age:.1f}s old)")
        if meta["rendered_point_count"] < meta["valid_point_count"]:
            warnings.append(
                f"downsampled from {meta['valid_point_count']} to "
                f"{meta['rendered_point_count']} points for rendering")
        return {
            "snapshot_id": artifact.snapshot_id,
            "capture_timestamp": meta["capture_timestamp"],
            "ros_message_stamp": meta["ros_message_stamp"],
            "message_age_at_capture_sec": meta["message_age_at_capture_sec"],
            "frame_id": meta["frame_id"],
            "topic": meta["topic"],
            "original_point_count": meta["original_point_count"],
            "valid_point_count": meta["valid_point_count"],
            "rendered_point_count": meta["rendered_point_count"],
            "image_size": meta["image_size"],
            "render_seconds": meta["render_seconds"],
            "views": list(VIEW_NAMES),
            "images": images,
            "download_zip": f"{base}/api/snapshot/{artifact.snapshot_id}/download.zip",
            "warnings": warnings,
        }

    @app.get("/api/snapshot/{snapshot_id}/download.zip")
    async def get_zip(snapshot_id: str):
        try:
            artifact = coordinator.registry.get(snapshot_id)
        except SnapshotNotFound as exc:
            raise HTTPException(status_code=404, detail=str(exc))
        data = await anyio.to_thread.run_sync(build_zip, artifact)
        return Response(
            content=data, media_type="application/zip",
            headers={"Content-Disposition":
                     f'attachment; filename="snapshot_{snapshot_id}.zip"'})

    @app.get("/api/snapshot/{snapshot_id}/{view}")
    async def get_view(snapshot_id: str, view: str):
        view_name = view[:-4] if view.endswith(".png") else view
        if view_name not in VIEW_NAMES:
            raise HTTPException(
                status_code=404,
                detail=f"unknown view {view!r}; expected one of {list(VIEW_NAMES)}")
        try:
            artifact = coordinator.registry.get(snapshot_id)
        except SnapshotNotFound as exc:
            raise HTTPException(status_code=404, detail=str(exc))
        path = artifact.files.get(view_name)
        if path is None or not path.is_file():
            raise HTTPException(status_code=404, detail="image not found")
        return FileResponse(path, media_type="image/png",
                            filename=f"{view_name}.png")

    # ------------------------------------------------------------------
    # Frontend
    # ------------------------------------------------------------------
    if STATIC_DIR.is_dir():
        app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True),
                  name="static")
    else:  # pragma: no cover
        @app.get("/")
        def index_missing():
            return JSONResponse({"error": "static frontend directory missing"},
                                status_code=500)

    return app
