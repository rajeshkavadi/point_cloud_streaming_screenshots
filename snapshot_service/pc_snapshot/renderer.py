"""Headless point-cloud PNG renderer.

Renders four camera views (top, side, front, perspective) of ONE immutable
snapshot using a shared bounding box and shared point colors.

Backend priority:
    1. Open3D OffscreenRenderer (works in WSL 2 via Vulkan/llvmpipe software
       rendering or a real GPU when present).
    2. Matplotlib (Agg) fallback when Open3D is unavailable or fails.

Coordinate conventions (configurable up_axis):
    up_axis="z" (ROS / robotics standard, e.g. FAST-LIO output in "camera_init"):
        top view    -> camera above  +Z, looking down -Z (image up = +Y)
        front view  -> camera at +X looking toward -X (image up = +Z)
        side view   -> camera at +Y looking toward -Y (image up = +Z)
        perspective -> oblique, ~45 deg elevation, configurable azimuth
    up_axis="y" (aerospace / some LiDAR raw frames):
        top view    -> camera above +Y; front view from +Z; side view from +X

All views share: same captured points, same colors, same bounding box.
"""

from __future__ import annotations

import io
import logging
import math
from typing import Dict, Optional, Tuple

import numpy as np
from PIL import Image, ImageDraw

from .config import Config
from .decoder import downsample_points
from .store import PointCloudSnapshot

log = logging.getLogger(__name__)

VIEW_NAMES = ("top", "side", "front", "perspective")


class RenderError(Exception):
    """Raised when rendering fails; message is safe to return to clients."""


# ---------------------------------------------------------------------------
# Camera geometry
# ---------------------------------------------------------------------------

def _camera_for_view(view: str, center: np.ndarray, dist: float,
                     up_axis: str, cfg: Config) -> Tuple[np.ndarray, np.ndarray]:
    """Return (eye_position, up_vector) for a named view."""
    e = math.radians(cfg.perspective_elev_deg)
    a = math.radians(cfg.perspective_azim_deg)
    if up_axis == "z":
        table = {
            "top": (center + np.array([0.0, 0.0, dist]), np.array([0.0, 1.0, 0.0])),
            "front": (center + np.array([dist, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])),
            "side": (center + np.array([0.0, dist, 0.0]), np.array([0.0, 0.0, 1.0])),
            "perspective": (
                center + dist * np.array([math.cos(e) * math.cos(a),
                                          math.cos(e) * math.sin(a),
                                          math.sin(e)]),
                np.array([0.0, 0.0, 1.0])),
        }
    else:  # y-up
        table = {
            "top": (center + np.array([0.0, dist, 0.0]), np.array([0.0, 0.0, 1.0])),
            "front": (center + np.array([0.0, 0.0, dist]), np.array([0.0, 1.0, 0.0])),
            "side": (center + np.array([dist, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])),
            "perspective": (
                center + dist * np.array([math.cos(e) * math.sin(a),
                                          math.sin(e),
                                          math.cos(e) * math.cos(a)]),
                np.array([0.0, 1.0, 0.0])),
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

def _render_open3d(prepared: PreparedCloud, snap: PointCloudSnapshot,
                   cfg: Config) -> Dict[str, np.ndarray]:
    try:
        import open3d as o3d
    except Exception as exc:
        raise RenderError(f"Open3D is not importable: {exc}")

    w, h = cfg.image_width, cfg.image_height
    try:
        ro = o3d.visualization.rendering.OffscreenRenderer(w, h)
    except Exception as exc:
        raise RenderError(f"Open3D OffscreenRenderer init failed: {exc}")

    try:
        bg = cfg.background_rgb()
        ro.scene.set_background([bg[0], bg[1], bg[2], 1.0])

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(prepared.points.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(prepared.colors.astype(np.float64))

        mat = o3d.visualization.rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        mat.point_size = float(cfg.point_size)
        ro.scene.add_geometry("cloud", pcd, mat)

        bbox = o3d.geometry.AxisAlignedBoundingBox(prepared.bbox_min.astype(np.float64),
                                                   prepared.bbox_max.astype(np.float64))
        # Distance that keeps the whole bbox inside a vertical-fov camera.
        # 1.05 = tight fit (slight margin) so the scene fills the frame.
        vfov = math.radians(cfg.fov_deg)
        dist = prepared.radius / math.tan(vfov / 2.0) * 1.05

        out: Dict[str, np.ndarray] = {}
        for view in VIEW_NAMES:
            eye, up = _camera_for_view(view, prepared.center, dist, cfg.up_axis, cfg)
            ro.setup_camera(cfg.fov_deg, bbox, eye, up)
            buf = ro.capture_screen_float_buffer(False)
            arr = (np.clip(np.asarray(buf), 0.0, 1.0) * 255.0).astype(np.uint8)
            if arr.shape[0] == 0 or arr.shape[1] == 0:
                raise RenderError("Open3D captured an empty image buffer")
            out[view] = _label_image(arr, view, snap, prepared.points.shape[0])
        return out
    except RenderError:
        raise
    except Exception as exc:
        raise RenderError(f"Open3D rendering failed: {exc}")
    finally:
        try:
            del ro
        except Exception:
            pass


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

    # azim/elev in degrees, matching the documented camera table for z-up data.
    cam = {
        "top": (89.9, -90.0),
        "front": (0.0, 0.0),
        "side": (0.0, 90.0),
        "perspective": (cfg.perspective_elev_deg, cfg.perspective_azim_deg),
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

def render_snapshot(snap: PointCloudSnapshot,
                    cfg: Config) -> Tuple[Dict[str, np.ndarray], int]:
    """Render all four views of one snapshot.

    Returns ({view_name: (H, W, 3) uint8 RGB array}, rendered_point_count).
    Raises RenderError with a client-safe message on failure.
    """
    if snap.points.shape[0] == 0:
        raise RenderError("snapshot contains no valid points")

    points_ds, intensity_ds = downsample_points(snap.points, snap.intensity,
                                                cfg.max_render_points)
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
    prepared = PreparedCloud(points_ds, intensity_ds, cfg)

    backend = cfg.renderer
    if backend in ("auto", "open3d"):
        try:
            images = _render_open3d(prepared, snap, cfg)
            log.info("rendered %s with Open3D backend", snap.seq)
            return images, prepared.points.shape[0]
        except RenderError as exc:
            if backend == "open3d":
                raise
            log.warning("Open3D backend failed (%s); falling back to Matplotlib", exc)
    return _render_matplotlib(prepared, snap, cfg), prepared.points.shape[0]


def images_to_png_bytes(images: Dict[str, np.ndarray]) -> Dict[str, bytes]:
    out = {}
    for view, arr in images.items():
        img = Image.fromarray(arr)
        bio = io.BytesIO()
        img.save(bio, format="PNG")
        out[view] = bio.getvalue()
    return out
