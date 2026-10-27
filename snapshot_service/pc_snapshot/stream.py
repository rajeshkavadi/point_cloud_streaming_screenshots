"""Live point-cloud streaming over multipart/x-mixed-replace.

Concept (same technique as Flask/OpenCV camera streaming tutorials):
    a background loop renders the LATEST point cloud to JPEG frames and
    publishes them; the HTTP endpoint yields each new frame in a multipart
    stream; the browser displays it in a plain <img> tag with no JavaScript
    video/WebSocket code needed.

Differences from the camera tutorial:
    * frame source = ROS PointCloud2 -> numpy -> Open3D offscreen render
      (instead of cv2.VideoCapture -> JPEG)
    * a persistent OffscreenRenderer is reused across frames (re-initializing
      Vulkan costs ~1s/frame, which would make streaming impossible)
    * the render loop only runs while at least one client is connected and
      stops after stream_idle_stop_sec of no clients (bounded resource use)
    * per-client camera view (top/front/back/left/right) is supported; the
      loop renders exactly the views currently being watched

The manager is transport-agnostic: FastAPI's StreamingResponse consumes
wait_for_frame(); unit tests can inject a stub render_fn.
"""

from __future__ import annotations

import io
import logging
import threading
import time
from typing import Callable, Dict, Optional, Tuple

import numpy as np
from PIL import Image, ImageDraw

from .config import Config
from .renderer import (RenderError, get_shared_renderer, prepare_cloud,
                       render_snapshot, render_views_offscreen, submit_render)
from .store import SnapshotStore

log = logging.getLogger(__name__)

# (frame_seq, jpeg_bytes, meta) per view
Frame = Tuple[int, bytes, dict]

BOUNDARY = "frame"


def jpeg_bytes(arr: np.ndarray, quality: int) -> bytes:
    img = Image.fromarray(arr)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=int(quality))
    return buf.getvalue()


def multipart_part(jpeg: bytes) -> bytes:
    return (f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
            f"Content-Length: {len(jpeg)}\r\n\r\n").encode() + jpeg + b"\r\n"


def mjpeg_generator(mgr: "StreamManager", view: str,
                    wait_timeout: float = 3.0,
                    heartbeat_sec: float = 2.0):
    """Yield multipart parts forever (until the client disconnects).

    Frames newer than the last yielded one are streamed immediately; while
    waiting (e.g. no point cloud data yet) the placeholder JPEG is re-sent as
    a keep-alive, which also lets the browser show status. Client cleanup runs
    in `finally`, which the server triggers with GeneratorExit on disconnect.
    This is the exact mechanism Flask/OpenCV MJPEG camera tutorials use.
    """
    last_seq = 0
    heartbeat = 0.0
    try:
        while True:
            frame = mgr.wait_for_frame(view, last_seq, timeout=wait_timeout)
            if frame is None:
                now = time.time()
                if now - heartbeat >= heartbeat_sec:
                    heartbeat = now
                    yield multipart_part(mgr.placeholder_jpeg)
                continue
            seq, jpeg, meta = frame
            last_seq = seq
            yield multipart_part(jpeg)
    except GeneratorExit:
        raise
    finally:
        mgr.remove_client(view)


class StreamManager:
    """Renders the latest point cloud to JPEG frames for live clients."""

    def __init__(self, cfg: Config, store: SnapshotStore,
                 render_fn: Optional[Callable] = None):
        """render_fn(snap, views, cfg) -> {view: (H,W,3) uint8}.

        When None, the real Open3D/Matplotlib renderer is used (created
        lazily on the first render, inside the render thread).
        """
        self.cfg = cfg
        self.store = store
        self._render_fn = render_fn

        self._lock = threading.Lock()
        self._cond = threading.Condition()
        self._clients = 0
        self._views: Dict[str, int] = {}       # view -> watching clients
        self._frames: Dict[str, Frame] = {}
        self._seq = 0
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._last_leave: Optional[float] = None

        self.rendered_frames = 0
        self.last_error: Optional[str] = None
        self.last_fps: float = 0.0
        self.last_points: int = 0
        self.last_frame_time: float = 0.0

        self._placeholder = self._make_placeholder()

    # ------------------------------------------------------------------
    # Client lifecycle
    # ------------------------------------------------------------------
    def add_client(self, view: str) -> bool:
        """Register a viewer. Returns True if the render thread was started."""
        with self._lock:
            self._clients += 1
            self._views[view] = self._views.get(view, 0) + 1
            self._last_leave = None
            if self._thread is not None and self._thread.is_alive():
                return False
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop,
                                            name="stream-render", daemon=True)
            self._thread.start()
            log.info("stream started (view=%s)", view)
            return True

    def remove_client(self, view: str) -> None:
        with self._lock:
            self._clients = max(0, self._clients - 1)
            if view in self._views:
                self._views[view] -= 1
                if self._views[view] <= 0:
                    del self._views[view]
            if self._clients == 0:
                self._last_leave = time.time()

    @property
    def client_count(self) -> int:
        with self._lock:
            return self._clients

    # ------------------------------------------------------------------
    # Frame publication / consumption
    # ------------------------------------------------------------------
    def wait_for_frame(self, view: str, last_seq: int,
                       timeout: float = 3.0) -> Optional[Frame]:
        """Block until a frame newer than last_seq exists for `view`."""
        with self._cond:
            self._cond.wait_for(
                lambda: (view in self._frames and self._frames[view][0] > last_seq)
                or self._stop.is_set(),
                timeout=timeout)
            frame = self._frames.get(view)
            if frame is not None and frame[0] > last_seq:
                return frame
            return None

    @property
    def placeholder_jpeg(self) -> bytes:
        return self._placeholder

    def _publish(self, view: str, jpeg: bytes, meta: dict) -> None:
        with self._cond:
            self._seq += 1
            self._frames[view] = (self._seq, jpeg, meta)
            self._cond.notify_all()

    # ------------------------------------------------------------------
    # Render loop
    # ------------------------------------------------------------------
    def _loop(self) -> None:
        min_interval = 1.0 / max(0.1, self.cfg.stream_fps)
        announced = False
        try:
            while not self._stop.is_set():
                with self._lock:
                    views = list(self._views)
                    leaving = (self._clients == 0 and self._last_leave is not None
                               and time.time() - self._last_leave
                               > self.cfg.stream_idle_stop_sec)
                if leaving:
                    log.info("stream idle -> stopping render loop")
                    break
                if not views:
                    time.sleep(0.2)
                    continue
                if not announced:
                    # Show something immediately: the first real render can
                    # take seconds (Filament pipeline warm-up on first use).
                    self._publish_all(views, self._placeholder,
                                      {"status": "starting_renderer"})
                    announced = True

                snap = self.store.capture()
                if snap is None:
                    # No data yet: keep the placeholder visible.
                    self._publish_all(views, self._placeholder,
                                      {"status": "waiting_for_data"})
                    time.sleep(0.5)
                    continue

                t0 = time.time()
                try:
                    images, count = self._render(snap, views)
                except RenderError as exc:
                    self.last_error = str(exc)
                    log.warning("stream render failed: %s", exc)
                    self._publish_all(views, self._error_jpeg(str(exc)),
                                      {"status": "error", "error": str(exc)})
                    time.sleep(1.0)
                    continue
                except Exception as exc:  # pragma: no cover - defensive
                    self.last_error = f"stream render crashed: {exc}"
                    log.exception("stream render crashed")
                    self._publish_all(views, self._error_jpeg(str(exc)),
                                      {"status": "error", "error": str(exc)})
                    time.sleep(1.0)
                    continue

                dt = time.time() - t0
                self.last_error = None
                self.last_fps = round(1.0 / dt, 2) if dt > 0 else 0.0
                self.last_points = count
                self.last_frame_time = time.time()
                meta = {
                    "status": "ok",
                    "ros_stamp": snap.ros_stamp,
                    "frame_id": snap.frame_id,
                    "topic": snap.topic,
                    "points": count,
                    "render_seconds": round(dt, 3),
                    "fps": self.last_fps,
                }
                for view, arr in images.items():
                    self._publish(view, jpeg_bytes(arr,
                                                   self.cfg.stream_jpeg_quality),
                                  meta)
                self.rendered_frames += len(images)

                # Frame-rate cap so we don't out-run the target.
                elapsed = time.time() - t0
                if elapsed < min_interval:
                    time.sleep(min_interval - elapsed)
        except Exception:  # pragma: no cover - defensive
            log.exception("stream render loop died")
            self.last_error = "stream render loop died"
        finally:
            # NOTE: the shared OffscreenRenderer is intentionally NOT
            # destroyed here (see renderer module docstring).
            log.info("stream render loop exited")

    def _render(self, snap, views):
        """Render `views` of one snapshot. Returns ({view: arr}, count)."""
        if self._render_fn is not None:
            images = self._render_fn(snap, views, self.cfg)
            # stub contract: images only; count inferred from snapshot
            return images, snap.point_count

        prepared, count = prepare_cloud(
            snap, self.cfg, max_points=self.cfg.stream_max_points)
        backend = self.cfg.renderer
        if backend in ("auto", "open3d"):
            try:
                # Open3D runs on the dedicated render thread (Filament thread
                # affinity) at SNAPSHOT resolution, then we scale down for
                # stream bandwidth.
                images = submit_render(self._open3d_job, prepared, snap, views)
                return self._fit_stream_size(images), count
            except RenderError as exc:
                if backend == "open3d":
                    raise
                log.warning("stream Open3D failed (%s); using Matplotlib "
                            "fallback (slower)", exc)
        # Matplotlib fallback: renders all views at stream resolution.
        import dataclasses
        scfg = dataclasses.replace(self.cfg,
                                   image_width=self.cfg.stream_width,
                                   image_height=self.cfg.stream_height,
                                   max_render_points=self.cfg.stream_max_points)
        images = render_snapshot(snap, scfg)[0]
        return {v: images[v] for v in views if v in images}, count

    def _open3d_job(self, prepared, snap, views):
        """Runs ON the render thread (Filament thread affinity — see
        renderer.submit_render). Renders at snapshot resolution; the caller
        downscales to stream size."""
        ro = get_shared_renderer(self.cfg, self.cfg.image_width,
                                 self.cfg.image_height)
        return render_views_offscreen(ro, prepared, snap, self.cfg,
                                      views=views)

    def _fit_stream_size(self, images: Dict) -> Dict:
        """Downscale frames (renderer size -> stream size) for bandwidth."""
        tw, th = self.cfg.stream_width, self.cfg.stream_height
        out = {}
        for view, arr in images.items():
            if arr.shape[1] != tw or arr.shape[0] != th:
                arr = np.asarray(Image.fromarray(arr).resize((tw, th),
                                                             Image.BILINEAR))
            out[view] = arr
        return out

    def _publish_all(self, views, jpeg: bytes, meta: dict) -> None:
        for view in views:
            self._publish(view, jpeg, meta)

    # ------------------------------------------------------------------
    # Status + placeholders
    # ------------------------------------------------------------------
    def status(self) -> dict:
        with self._lock:
            clients = self._clients
            active = dict(self._views)
            thread_alive = self._thread is not None and self._thread.is_alive()
        return {
            "enabled": self.cfg.stream_enabled,
            "clients": clients,
            "active_views": active,
            "render_loop_alive": thread_alive,
            "rendered_frames": self.rendered_frames,
            "last_fps": self.last_fps,
            "last_points": self.last_points,
            "last_frame_age_sec":
                None if self.last_frame_time == 0
                else round(time.time() - self.last_frame_time, 2),
            "resolution": [self.cfg.stream_width, self.cfg.stream_height],
            "target_fps": self.cfg.stream_fps,
            "jpeg_quality": self.cfg.stream_jpeg_quality,
            "last_error": self.last_error,
        }

    @staticmethod
    def _text_frame(title: str, line1: str, line2: str = "") -> Image.Image:
        img = Image.new("RGB", (960, 540), (13, 16, 23))
        d = ImageDraw.Draw(img)
        d.text((40, 230), title, fill=(219, 226, 238))
        d.text((40, 255), line1, fill=(139, 150, 168))
        if line2:
            d.text((40, 275), line2, fill=(139, 150, 168))
        return img

    def _make_placeholder(self) -> bytes:
        img = self._text_frame("Waiting for point cloud data...",
                               "Start bag playback / sensor driver, then refresh.")
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=70)
        return buf.getvalue()

    def _error_jpeg(self, msg: str) -> bytes:
        img = self._text_frame("Stream render error", msg[:110],
                               "See host service log for details.")
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=70)
        return buf.getvalue()
