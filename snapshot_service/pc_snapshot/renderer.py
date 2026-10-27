"""Headless point-cloud PNG renderer.

Renders five camera views (top, front, back, left, right) of ONE immutable
snapshot using a shared bounding box and shared point colors.

Backend priority:
    1. Open3D OffscreenRenderer (works in WSL 2 via Vulkan/llvmpipe software
       rendering or a real GPU when present).
    2. Matplotlib (Agg) fallback when Open3D is unavailable or fails.

Thread-affinity model (IMPORTANT):
    This Open3D/Filament build enforces thread adoption — a renderer may only
    be USED on the thread that created it ("This thread has not been
    adopted"), destroying one while another exists panics the process, and
    every freshly created renderer re-pays a ~5s first-render warm-up.
    Therefore: exactly ONE OffscreenRenderer exists, it is created and used
    exclusively on a single dedicated render thread, and every Open3D call
    (snapshot renders, live-stream frames, tests) must be submitted to that
    thread via submit_render(). The renderer is destroyed only once, on that
    same thread, during shutdown_render_thread() (also registered via atexit).

Coordinate conventions (configurable up_axis):
    front/back/left/right are FIRST-PERSON: the eye sits exactly at the
    sensor origin (0,0,0 — the Livox Mid-360 / Ouster position in its own
    frame) and looks outward, as if you were standing at the sensor.
    up_axis="z" (ROS / robotics standard, e.g. FAST-LIO output in "camera_init"):
        top view    -> camera above  +Z, looking down -Z (image up = +Y)
        front view  -> eye at origin, looking +X (image up = +Z)
        back view   -> eye at origin, looking -X
        left view   -> eye at origin, looking +Y
        right view  -> eye at origin, looking -Y
    up_axis="y" (aerospace / some LiDAR raw frames):
        top view    -> camera above +Y; front looks +Z; back looks -Z;
                       left looks +X; right looks -X (image up = +Y)

All views share: same captured points, same colors, same bounding box.
"""

from __future__ import annotations

import atexit
import concurrent.futures
import gc
import io
import logging
import math
import queue
import threading
from typing import Dict, Optional, Tuple

import numpy as np
from PIL import Image, ImageDraw

from .config import VIEW_NAMES, Config
from .decoder import downsample_points
from .store import PointCloudSnapshot

log = logging.getLogger(__name__)

# VIEW_NAMES is re-exported from .config (single source of truth).
__all__ = ["VIEW_NAMES", "RenderError", "render_snapshot", "prepare_cloud",
           "images_to_png_bytes", "submit_render", "shutdown_render_thread",
           "get_shared_renderer", "make_offscreen_renderer",
           "render_views_offscreen"]

# ---------------------------------------------------------------------------
# Dedicated render thread (owns THE OffscreenRenderer)
# ---------------------------------------------------------------------------
# Job = (callable, args, Future) or None (shutdown sentinel).
_render_queue: Optional["queue.Queue"] = None
_render_thread: Optional[threading.Thread] = None
_shutdown_done = False
_state_lock = threading.Lock()

# The one and only OffscreenRenderer — ONLY touched on the render thread.
_SHARED_RO = None
_SHARED_SIZE: Optional[Tuple[int, int]] = None

# Soft bound: snapshot jobs are limited elsewhere; the stream submits one job
# at a time, so a healthy service never approaches this.
MAX_RENDER_QUEUE = 16


def _render_worker() -> None:
    while True:
        job = _render_queue.get()
        if job is None:
            break
        fn, args, fut = job
        try:
            result = fn(*args)
        except BaseException as exc:      # keep the thread alive regardless
            try:
                fut.set_exception(exc)
            except Exception:
                pass
        else:
            try:
                fut.set_result(result)
            except Exception:             # cancelled future — ignore
                pass


def submit_render(fn, *args):
    """Run fn(*args) on THE render thread; return its result (or raise).

    All Open3D/Filament calls must go through here — see module docstring.
    The queue put happens under _state_lock so a job can never slip in
    between shutdown's destroy-job and its sentinel.
    """
    global _render_queue, _render_thread
    fut = concurrent.futures.Future()
    with _state_lock:
        if _shutdown_done:
            raise RenderError("render thread already shut down")
        if _render_queue is None:
            _render_queue = queue.Queue()
        if _render_thread is None or not _render_thread.is_alive():
            _render_thread = threading.Thread(target=_render_worker,
                                              name="o3d-render",
                                              daemon=True)
            _render_thread.start()
        if _render_queue.qsize() >= MAX_RENDER_QUEUE:
            raise RenderError(
                f"render queue overloaded ({_render_queue.qsize()} pending)")
        _render_queue.put((fn, args, fut))
    return fut.result()


def shutdown_render_thread(timeout: float = 15.0) -> None:
    """Destroy the shared renderer ON its owning thread, then stop it.

    Idempotent; called from __main__ and via atexit so interpreter exit
    never destroys the renderer from a foreign thread (which panics).
    """
    global _render_queue, _render_thread, _SHARED_RO, _SHARED_SIZE, _shutdown_done
    with _state_lock:
        if _shutdown_done:
            return
        _shutdown_done = True
        th, q = _render_thread, _render_queue
    if th is None or not th.is_alive() or q is None:
        return

    def _destroy_on_owner() -> None:
        global _SHARED_RO, _SHARED_SIZE
        _SHARED_RO = None
        _SHARED_SIZE = None
        gc.collect()

    fut = concurrent.futures.Future()
    q.put((_destroy_on_owner, (), fut))
    try:
        fut.result(timeout=timeout)
    except Exception as exc:  # pragma: no cover - defensive
        log.warning("renderer destroy job failed: %s", exc)
    q.put(None)               # sentinel: worker exits
    try:
        th.join(timeout=timeout)
    except RuntimeError:  # pragma: no cover
        pass
    with _state_lock:
        _render_thread = None
        _render_queue = None


atexit.register(shutdown_render_thread)


class RenderError(Exception):
    """Raised when rendering fails; message is safe to return to clients."""


# ---------------------------------------------------------------------------
# Camera geometry
# ---------------------------------------------------------------------------

def _camera_for_view(view: str, center: np.ndarray, dist: float,
                     up_axis: str, cfg: Config
                     ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Return (eye, look_at_target, up, near_clip) for a named view.

    front/back/left/right are FIRST-PERSON views: the eye sits exactly at
    the sensor origin (0,0,0 — the Livox Mid-360 / Ouster position in its
    own frame) and looks outward in that direction, as if you were standing
    at the sensor. top stays an overhead framing of the cloud bounds.
    """
    origin = np.zeros(3)
    if up_axis == "z":
        table = {
            # overhead: framed on the cloud bounds
            "top": (center + np.array([0.0, 0.0, dist]), center,
                    np.array([0.0, 1.0, 0.0]), -1.0),
            # first-person (x forward, y left, z up): eye at the sensor
            "front": (origin, np.array([dist, 0.0, 0.0]),
                      np.array([0.0, 0.0, 1.0]), 0.05),
            "back": (origin, np.array([-dist, 0.0, 0.0]),
                     np.array([0.0, 0.0, 1.0]), 0.05),
            "left": (origin, np.array([0.0, dist, 0.0]),
                     np.array([0.0, 0.0, 1.0]), 0.05),
            "right": (origin, np.array([0.0, -dist, 0.0]),
                      np.array([0.0, 0.0, 1.0]), 0.05),
        }
    else:  # y-up (aerospace / some raw sensor frames: z forward, y up)
        table = {
            "top": (center + np.array([0.0, dist, 0.0]), center,
                    np.array([0.0, 0.0, 1.0]), -1.0),
            "front": (origin, np.array([0.0, 0.0, dist]),
                      np.array([0.0, 1.0, 0.0]), 0.05),
            "back": (origin, np.array([0.0, 0.0, -dist]),
                     np.array([0.0, 1.0, 0.0]), 0.05),
            "left": (origin, np.array([dist, 0.0, 0.0]),
                     np.array([0.0, 1.0, 0.0]), 0.05),
            "right": (origin, np.array([-dist, 0.0, 0.0]),
                      np.array([0.0, 1.0, 0.0]), 0.05),
        }
    if view not in table:
        raise RenderError(f"unknown view {view!r}; expected one of {VIEW_NAMES}")
    return table[view]


def compute_bounds(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Return (min, max, center, radius) using robust percentile bounds.

    Percentile-based (10%..90% per axis) bounds keep a handful of sparse,
    distant outlier points from stretching the camera fit so the main scene
    fills the frame.
    """
    pmin = np.percentile(points, 10.0, axis=0).astype(np.float64)
    pmax = np.percentile(points, 90.0, axis=0).astype(np.float64)
    # Guard against degenerate (flat) axes.
    extent = np.maximum(pmax - pmin, 1e-2)
    center = (pmin + pmax) / 2.0
    radius = float(0.5 * np.linalg.norm(extent))
    radius = max(radius, 0.1)
    pad = 0.08 * extent
    return pmin - pad, pmax + pad, center, radius


# ---------------------------------------------------------------------------
# Coloring
# ---------------------------------------------------------------------------

_HEIGHT_COLORS = np.array([
    [0.05, 0.15, 0.65],   # low: blue
    [0.0, 0.75, 0.85],    # cyan
    [0.1, 0.85, 0.2],     # green
    [0.95, 0.85, 0.1],    # yellow
    [0.9, 0.15, 0.1],     # high: red
], dtype=np.float32)


def _colormap(values01: np.ndarray) -> np.ndarray:
    """Map values in [0,1] to RGB using a fixed 5-stop gradient."""
    v = np.clip(values01, 0.0, 1.0)
    n = len(_HEIGHT_COLORS) - 1
    pos = v * n
    idx = np.floor(pos).astype(np.int32)
    idx = np.clip(idx, 0, n - 1)
    frac = (pos - idx)[:, None].astype(np.float32)
    c0 = _HEIGHT_COLORS[idx]
    c1 = _HEIGHT_COLORS[idx + 1]
    return c0 + frac * (c1 - c0)


def compute_colors(points: np.ndarray, intensity: Optional[np.ndarray],
                   mode: str) -> np.ndarray:
    """Return (N,3) float32 colors in [0,1] for the given snapshot."""
    if points.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32)
    axis = 2  # height = Z for z-up; callers pass already-remapped points
    vals = points[:, axis].astype(np.float64)
    if mode == "intensity" and intensity is not None and intensity.shape[0] == points.shape[0]:
        vals = intensity.astype(np.float64)
    lo, hi = np.percentile(vals, [2.0, 98.0])
    if not np.isfinite(lo) or not np.isfinite(hi) or hi - lo < 1e-9:
        return np.full((points.shape[0], 3), 0.55, dtype=np.float32)
    norm = (vals - lo) / (hi - lo)
    return _colormap(norm)


# ---------------------------------------------------------------------------
# Shared preparation
# ---------------------------------------------------------------------------

class PreparedCloud:
    """Points + colors + bounds prepared once and reused by every view."""

    def __init__(self, points_ds: np.ndarray, intensity_ds: Optional[np.ndarray],
                 cfg: Config):
        # Work on a private writable copy so the read-only snapshot arrays
        # are never mutated and per-backend transforms (y-up remap) are safe.
        pts = np.array(points_ds, dtype=np.float32, copy=True)
        if cfg.up_axis == "y":
            # Remap so the up axis becomes Z for coloring math only.
            color_pts = pts[:, [0, 2, 1]]
        else:
            color_pts = pts
        self.points = pts
        self.colors = compute_colors(color_pts, intensity_ds, cfg.color_mode)
        self.bbox_min, self.bbox_max, self.center, self.radius = compute_bounds(pts)


# ---------------------------------------------------------------------------
# Label overlay (shared by both backends)
# ---------------------------------------------------------------------------

def _label_image(arr: np.ndarray, view: str, snap: PointCloudSnapshot,
                 rendered_points: int) -> np.ndarray:
    img = Image.fromarray(arr)
    draw = ImageDraw.Draw(img, "RGBA")
    stamp = snap.ros_stamp if snap.ros_stamp > 0 else snap.received_walltime
    try:
        import datetime
        stamp_str = datetime.datetime.utcfromtimestamp(stamp).strftime("%H:%M:%S.%f")[:-3] + "Z"
    except (OverflowError, OSError, ValueError):
        stamp_str = "unknown"
    title = f"{view.upper()}"
    meta = (f"frame: {snap.frame_id or 'n/a'}  |  ros stamp: {stamp_str}  |  "
            f"{rendered_points:,} pts  |  topic: {snap.topic or 'n/a'}")
    bar_h = 42
    draw.rectangle([0, 0, img.width, bar_h], fill=(0, 0, 0, 150))
    draw.text((12, 6), title, fill=(255, 255, 255, 255))
    draw.text((12, 23), meta, fill=(200, 210, 220, 255))
    return np.asarray(img)


# ---------------------------------------------------------------------------
# Open3D backend
# ---------------------------------------------------------------------------

def _import_open3d():
    try:
        import open3d as o3d
        return o3d
    except Exception as exc:
        raise RenderError(f"Open3D is not importable: {exc}")


def make_offscreen_renderer(cfg: Config, width: int = None,
                            height: int = None):
    """Create an Open3D OffscreenRenderer (background applied).

    LOW-LEVEL: prefer get_shared_renderer() — creating a second renderer
    while another exists, then destroying either, panics this Filament build.
    width/height default to cfg.image_width/cfg.image_height.
    """
    o3d = _import_open3d()
    try:
        ro = o3d.visualization.rendering.OffscreenRenderer(
            width or cfg.image_width, height or cfg.image_height)
        bg = cfg.background_rgb()
        ro.scene.set_background([bg[0], bg[1], bg[2], 1.0])
        return ro
    except Exception as exc:
        raise RenderError(f"Open3D OffscreenRenderer init failed: {exc}")


def get_shared_renderer(cfg: Config, width: int = None,
                        height: int = None):
    """Return THE process-wide OffscreenRenderer at the requested size.

    MUST run on the render thread (only via submit_render jobs) — Filament
    thread affinity. On a size change the old renderer is dropped BEFORE
    creating the new one so two renderers are never alive at the same time
    (that is what trips the Filament "MaterialInstance still in use" panic).
    Callers must not retain renderer references across size changes.
    """
    global _SHARED_RO, _SHARED_SIZE
    w = int(width or cfg.image_width)
    h = int(height or cfg.image_height)
    if _SHARED_RO is not None and _SHARED_SIZE == (w, h):
        return _SHARED_RO
    if _SHARED_RO is not None:
        _SHARED_RO = None          # destroy old FIRST (single-instance rule)
        gc.collect()
    _SHARED_RO = make_offscreen_renderer(cfg, width=w, height=h)
    _SHARED_SIZE = (w, h)
    log.info("shared OffscreenRenderer ready at %dx%d", w, h)
    return _SHARED_RO


def render_views_offscreen(ro, prepared: PreparedCloud, snap: PointCloudSnapshot,
                           cfg: Config, views=VIEW_NAMES) -> Dict[str, np.ndarray]:
    """Render the given views with an already-created OffscreenRenderer."""
    o3d = _import_open3d()
    try:
        if ro.scene.has_geometry("cloud"):
            ro.scene.remove_geometry("cloud")
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(prepared.points.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(prepared.colors.astype(np.float64))
        mat = o3d.visualization.rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        mat.point_size = float(cfg.point_size)
        ro.scene.add_geometry("cloud", pcd, mat)

        # Distance that keeps the whole bbox inside a vertical-fov camera.
        # 1.05 = tight fit (slight margin) so the scene fills the frame.
        vfov = math.radians(cfg.fov_deg)
        dist = prepared.radius / math.tan(vfov / 2.0) * 1.05

        out: Dict[str, np.ndarray] = {}
        for view in views:
            eye, target, up, near = _camera_for_view(view, prepared.center,
                                                     dist, cfg.up_axis, cfg)
            # Open3D 0.20 OffscreenRenderer.setup_camera takes the camera
            # CENTER/look-at (ArrayLike), not an AxisAlignedBoundingBox —
            # passing the bbox raises TypeError (which used to silently fall
            # back to Matplotlib in `auto` mode and hide this).
            # First-person views pass an explicit small near clip so points
            # right next to the sensor are not clipped away.
            ro.setup_camera(cfg.fov_deg, target, eye, up,
                            near, float(dist + prepared.radius * 2.0))
            # This build exposes render_to_image() (H, W, 3) uint8 instead of
            # capture_screen_float_buffer(). Copy immediately: the numpy view
            # may alias an internal buffer that the next render reuses, and
            # consumers live on other threads.
            img = ro.render_to_image()
            arr = np.array(img)
            if arr.ndim == 2:
                arr = np.stack([arr] * 3, axis=-1)
            if arr.shape[-1] == 4:
                arr = arr[:, :, :3]
            arr = np.ascontiguousarray(arr, dtype=np.uint8)
            if arr.shape[0] == 0 or arr.shape[1] == 0:
                raise RenderError("Open3D captured an empty image buffer")
            out[view] = _label_image(arr, view, snap, prepared.points.shape[0])
        return out
    except RenderError:
        raise
    except Exception as exc:
        raise RenderError(f"Open3D rendering failed: {exc}")


def _render_open3d(prepared: PreparedCloud, snap: PointCloudSnapshot,
                   cfg: Config) -> Dict[str, np.ndarray]:
    # Open3D runs ONLY on the dedicated render thread (thread affinity).
    _import_open3d()          # fail fast before starting the thread
    return submit_render(_render_open3d_job, prepared, snap, cfg)


def _render_open3d_job(prepared: PreparedCloud, snap: PointCloudSnapshot,
                       cfg: Config) -> Dict[str, np.ndarray]:
    ro = get_shared_renderer(cfg, cfg.image_width, cfg.image_height)
    return render_views_offscreen(ro, prepared, snap, cfg, VIEW_NAMES)


# ---------------------------------------------------------------------------
# Matplotlib fallback backend
# ---------------------------------------------------------------------------

def _render_matplotlib(prepared: PreparedCloud, snap: PointCloudSnapshot,
                       cfg: Config) -> Dict[str, np.ndarray]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
    except Exception as exc:
        raise RenderError(f"Matplotlib fallback unavailable: {exc}")

    w, h = cfg.image_width, cfg.image_height
    bg = cfg.background_rgb()

    pts = prepared.points
    colors = prepared.colors
    up_swap = cfg.up_axis == "y"
    if up_swap:
        pts = pts[:, [0, 2, 1]]  # bring up-axis (y) to matplotlib's Z

    # Direction of view in degrees (elev, azim) for matplotlib's ORBIT-style
    # camera — it cannot sit inside the cloud, so first-person views are
    # approximated by placing the camera behind the sensor on the opposite
    # side (same look direction). z-up data: azim 0 = camera at +X.
    cam = {
        "top": (89.9, -90.0),
        "front": (0.0, 180.0),   # camera behind origin at -X, looking +X
        "back": (0.0, 0.0),      # camera at +X, looking -X
        "left": (0.0, -90.0),    # camera at -Y, looking +Y
        "right": (0.0, 90.0),    # camera at +Y, looking -Y
    }
    pmin = pts.min(axis=0)
    pmax = pts.max(axis=0)
    extents = np.maximum(pmax - pmin, 1e-2)

    out: Dict[str, np.ndarray] = {}
    for view in VIEW_NAMES:
        fig = Figure(figsize=(w / 100.0, h / 100.0), dpi=100)
        FigureCanvasAgg(fig)
        fig.patch.set_facecolor(bg)
        ax = fig.add_subplot(111, projection="3d", facecolor=bg)
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=colors,
                   s=max(0.5, cfg.point_size * 0.6), marker=".", linewidths=0,
                   rasterized=True)
        elev, azim = cam[view]
        ax.view_init(elev=elev, azim=azim)
        ax.set_box_aspect(extents.tolist())
        ax.set_axis_off()
        try:
            fig.subplots_adjust(left=-0.15, right=1.15, top=1.15, bottom=-0.15)
        except Exception:
            pass
        buf = io.BytesIO()
        fig.savefig(buf, format="png", facecolor=bg, dpi=100)
        plt.close(fig)
        buf.seek(0)
        img = np.asarray(Image.open(buf).convert("RGB"))
        if img.shape[0] < h or img.shape[1] < w:
            pad = np.zeros((h, w, 3), dtype=np.uint8)
            pad[:img.shape[0], :img.shape[1]] = img
            img = pad
        out[view] = _label_image(img[:h, :w], view, snap, prepared.points.shape[0])
    return out


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def prepare_cloud(snap: PointCloudSnapshot, cfg: Config,
                  max_points: int = None) -> Tuple[PreparedCloud, int]:
    """Downsample + clip + color + bounds one snapshot for rendering.

    Returns (PreparedCloud, rendered_point_count).
    Raises RenderError when nothing usable remains.
    """
    if snap.points.shape[0] == 0:
        raise RenderError("snapshot contains no valid points")
    if max_points is None:
        max_points = cfg.max_render_points

    points_ds, intensity_ds = downsample_points(snap.points, snap.intensity,
                                                max_points)
    # Optional radial clip from the sensor origin (useful for raw LiDAR with
    # sparse far returns that would otherwise dominate the camera fit).
    if cfg.clip_range_m > 0.0:
        r = np.linalg.norm(points_ds, axis=1)
        keep = r <= cfg.clip_range_m
        if keep.any():
            points_ds = points_ds[keep]
            if intensity_ds is not None:
                intensity_ds = intensity_ds[keep]
    if points_ds.shape[0] == 0:
        raise RenderError("downsampling produced an empty cloud (internal error)")
    return PreparedCloud(points_ds, intensity_ds, cfg), points_ds.shape[0]


def render_snapshot(snap: PointCloudSnapshot,
                    cfg: Config) -> Tuple[Dict[str, np.ndarray], int]:
    """Render all five views of one snapshot.

    Returns ({view_name: (H, W, 3) uint8 RGB array}, rendered_point_count).
    Raises RenderError with a client-safe message on failure.
    """
    prepared, count = prepare_cloud(snap, cfg)

    backend = cfg.renderer
    if backend in ("auto", "open3d"):
        try:
            images = _render_open3d(prepared, snap, cfg)
            log.info("rendered %s with Open3D backend", snap.seq)
            return images, count
        except RenderError as exc:
            if backend == "open3d":
                raise
            log.warning("Open3D backend failed (%s); falling back to Matplotlib", exc)
    return _render_matplotlib(prepared, snap, cfg), count


def images_to_png_bytes(images: Dict[str, np.ndarray]) -> Dict[str, bytes]:
    out = {}
    for view, arr in images.items():
        img = Image.fromarray(arr)
        bio = io.BytesIO()
        img.save(bio, format="PNG")
        out[view] = bio.getvalue()
    return out
