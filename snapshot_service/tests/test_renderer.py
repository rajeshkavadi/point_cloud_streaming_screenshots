"""Renderer tests: four views from one snapshot, camera geometry, PNG validity.

Uses the Open3D backend when it initializes; the camera-geometry tests are
backend-independent.
"""

import math

import numpy as np
import pytest

from pc_snapshot.config import Config
from pc_snapshot.decoder import decode_pointcloud2
from pc_snapshot.renderer import (RenderError, VIEW_NAMES, _camera_for_view,
                                  compute_bounds, compute_colors,
                                  images_to_png_bytes, render_snapshot)
from tests.fixtures import make_scene_cloud, make_xyz_cloud


def _snapshot(n=5000, **kw):
    from tests.fixtures import snapshot_from_cloud
    return snapshot_from_cloud(make_scene_cloud(n), seq=1)


@pytest.fixture(scope="module")
def cfg():
    c = Config()
    c.image_width = 640
    c.image_height = 360
    c.max_render_points = 50_000
    c.validate()
    return c


def test_camera_views_z_up(cfg):
    center = np.zeros(3)
    seen = {}
    for view in VIEW_NAMES:
        eye, up = _camera_for_view(view, center, 10.0, "z", cfg)
        seen[view] = (eye, up)
        assert eye.shape == (3,) and up.shape == (3,)
        assert not np.allclose(eye, center), "eye must not coincide with center"
    # Top: camera directly above (+Z), looking down.
    assert np.allclose(seen["top"][0], [0, 0, 10.0])
    # Front: from +X; side: from +Y.
    assert np.allclose(seen["front"][0], [10.0, 0, 0])
    assert np.allclose(seen["side"][0], [0, 10.0, 0])
    # Front and side use Z as image-up (Z is vertical in this convention).
    assert np.allclose(seen["front"][1], [0, 0, 1])
    assert np.allclose(seen["side"][1], [0, 0, 1])
    # Perspective is oblique ~45 degrees elevation.
    eye = seen["perspective"][0]
    elev = math.degrees(math.asin(eye[2] / np.linalg.norm(eye)))
    assert 40.0 <= elev <= 50.0
    # All four eyes differ.
    eyes = [tuple(np.round(e, 5)) for e, _ in seen.values()]
    assert len(set(eyes)) == 4


def test_camera_views_y_up(cfg):
    center = np.zeros(3)
    top_eye, _ = _camera_for_view("top", center, 10.0, "y", cfg)
    front_eye, front_up = _camera_for_view("front", center, 10.0, "y", cfg)
    assert np.allclose(top_eye, [0, 10.0, 0])      # above along +Y
    assert np.allclose(front_eye, [0, 0, 10.0])    # front is +Z when Y is up
    assert np.allclose(front_up, [0, 1.0, 0])      # image-up is the vertical axis
    with pytest.raises(RenderError):
        _camera_for_view("bogus", center, 10.0, "y", cfg)


def test_bounds_and_colors():
    pts = np.array([[0, 0, 0], [10, 5, 2]], dtype=np.float32)
    pmin, pmax, center, radius = compute_bounds(pts)
    assert radius > 0
    np.testing.assert_allclose(center, [5, 2.5, 1.0], atol=0.5)
    colors = compute_colors(pts, None, "height")
    assert colors.shape == (2, 3)
    assert colors[0].sum() < colors[1].sum()  # higher Z -> warmer color


def test_render_four_views_same_snapshot(cfg):
    snap = _snapshot()
    images, rendered_count = render_snapshot(snap, cfg)
    assert set(images.keys()) == set(VIEW_NAMES)
    assert rendered_count == snap.point_count
    for view in VIEW_NAMES:
        arr = images[view]
        assert arr.shape == (cfg.image_height, cfg.image_width, 3)
        assert arr.dtype == np.uint8
        # Not blank: some variance in the image.
        assert arr.std() > 1.0, f"{view} view looks blank (std={arr.std()})"
    # Different camera angles produce different images.
    top = images["top"]
    front = images["front"]
    assert not np.array_equal(top, front), "top and front views are identical"


def test_png_bytes_valid(cfg):
    snap = _snapshot(2000)
    images, _ = render_snapshot(snap, cfg)
    pngs = images_to_png_bytes(images)
    for view, data in pngs.items():
        assert data[:8] == b"\x89PNG\r\n\x1a\n", f"{view} is not a valid PNG"
        assert len(data) > 1000


def test_intensity_color_mode(cfg):
    cfg2 = Config()
    cfg2.image_width = 320
    cfg2.image_height = 180
    cfg2.color_mode = "intensity"
    cfg2.renderer = cfg.renderer
    cfg2.max_render_points = 50_000
    cfg2.validate()
    snap = _snapshot(2000)
    images, _ = render_snapshot(snap, cfg2)
    assert set(images.keys()) == set(VIEW_NAMES)


def test_empty_cloud_raises(cfg):
    from pc_snapshot.store import PointCloudSnapshot
    import time
    snap = PointCloudSnapshot(
        points=np.zeros((0, 3), dtype=np.float32), intensity=None,
        frame_id="", ros_stamp=0.0, received_walltime=time.time(),
        topic="/t", total_points_in_message=0, seq=1)
    with pytest.raises(RenderError):
        render_snapshot(snap, cfg)


def test_single_point_cloud(cfg):
    from pc_snapshot.store import PointCloudSnapshot
    import time
    snap = PointCloudSnapshot(
        points=np.array([[1.0, 2.0, 3.0]], dtype=np.float32), intensity=None,
        frame_id="f", ros_stamp=0.0, received_walltime=time.time(),
        topic="/t", total_points_in_message=1, seq=1)
    images, _ = render_snapshot(snap, cfg)
    assert set(images.keys()) == set(VIEW_NAMES)


def test_downsampled_render_count(cfg):
    snap = _snapshot(20_000)
    cfg_small = Config()
    cfg_small.image_width = 320
    cfg_small.image_height = 180
    cfg_small.max_render_points = 5_000
    cfg_small.renderer = cfg.renderer
    cfg_small.validate()
    images, rendered_count = render_snapshot(snap, cfg_small)
    assert rendered_count <= 5_000
    assert rendered_count < snap.point_count
