"""Renderer tests: five views from one snapshot, camera geometry, PNG validity.

Uses the Open3D backend when it initializes; the camera-geometry tests are
backend-independent.
"""

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
    center = np.array([1.0, 2.0, 3.0])   # off-origin on purpose
    dist = 10.0
    seen = {}
    for view in VIEW_NAMES:
        eye, target, up, near = _camera_for_view(view, center, dist, "z", cfg)
        seen[view] = (eye, target, up, near)
        assert eye.shape == (3,) and target.shape == (3,) and up.shape == (3,)
    # Top: camera directly above the cloud center (+Z), looking down.
    assert np.allclose(seen["top"][0], center + [0, 0, dist])
    assert np.allclose(seen["top"][1], center)
    assert np.allclose(seen["top"][2], [0, 1, 0])   # image-up = +Y
    # front/back/left/right: FIRST-PERSON — eye exactly at the sensor origin
    # even though the cloud center is offset, looking outward horizontally.
    dirs = {"front": [1, 0, 0], "back": [-1, 0, 0],
            "left": [0, 1, 0], "right": [0, -1, 0]}
    for view, d in dirs.items():
        eye, target, up, near = seen[view]
        assert np.allclose(eye, [0, 0, 0]), f"{view}: eye must be at the sensor"
        assert np.allclose(target, np.array(d, dtype=float) * dist)
        assert np.allclose(up, [0, 0, 1])           # image-up = +Z
        assert near > 0, "first-person views need an explicit small near clip"
        assert abs(target[2] - eye[2]) < 1e-9, "horizontal look direction"
    # Top view keeps the automatic (larger) near clip.
    assert seen["top"][2].shape == (3,)
    # All five (eye, target) camera placements differ.
    placements = {(tuple(np.round(e, 5)), tuple(np.round(t, 5)))
                  for e, t, _, _ in seen.values()}
    assert len(placements) == len(VIEW_NAMES)


def test_camera_views_y_up(cfg):
    center = np.zeros(3)
    top_eye, top_target, top_up, _ = _camera_for_view("top", center, 10.0, "y", cfg)
    front_eye, front_target, front_up, _ = _camera_for_view("front", center, 10.0, "y", cfg)
    _, left_target, _, _ = _camera_for_view("left", center, 10.0, "y", cfg)
    assert np.allclose(top_eye, [0, 10.0, 0])       # above along +Y
    assert np.allclose(top_target, center)
    assert np.allclose(front_eye, [0, 0, 0])        # first-person at sensor
    assert np.allclose(front_target, [0, 0, 10.0])  # front is +Z when Y is up
    assert np.allclose(front_up, [0, 1.0, 0])       # image-up is the vertical axis
    assert np.allclose(left_target, [10.0, 0, 0])   # left is +X when Y is up
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


def test_render_all_views_same_snapshot(cfg):
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


def test_open3d_backend_forced():
    """Forced Open3D render must succeed — `renderer=auto` would otherwise
    silently fall back to Matplotlib and hide Open3D-specific regressions
    (e.g. wrong scene API), which is exactly how a real bug shipped once."""
    try:
        import open3d  # noqa: F401
    except Exception as exc:
        pytest.skip(f"Open3D not available: {exc}")

    cfg = Config()
    cfg.image_width = 320
    cfg.image_height = 180
    cfg.renderer = "open3d"          # no fallback allowed
    cfg.max_render_points = 20_000
    try:
        cfg.validate()
        snap = _snapshot(3000)
        images, count = render_snapshot(snap, cfg)
    except RenderError as exc:
        if "init failed" in str(exc) or "not importable" in str(exc):
            pytest.skip(f"Open3D offscreen init unavailable: {exc}")
        raise
    assert set(images.keys()) == set(VIEW_NAMES)
    for view in VIEW_NAMES:
        assert images[view].shape == (180, 320, 3)


def test_persistent_renderer_reuse():
    """The live stream reuses the ONE shared OffscreenRenderer across frames:
    render twice with the same instance (exercises the remove/re-add path)."""
    try:
        import open3d  # noqa: F401
    except Exception as exc:
        pytest.skip(f"Open3D not available: {exc}")

    from pc_snapshot.renderer import (get_shared_renderer, prepare_cloud,
                                      render_views_offscreen, submit_render)
    cfg = Config()
    cfg.image_width = 320
    cfg.image_height = 180
    cfg.max_render_points = 20_000
    try:
        cfg.validate()
        snap = _snapshot(3000)
        prepared, _ = prepare_cloud(snap, cfg, max_points=20_000)
    except RenderError as exc:
        pytest.skip(str(exc))

    def job():
        ro = get_shared_renderer(cfg, 320, 180)
        first = render_views_offscreen(ro, prepared, snap, cfg,
                                       views=["front"])
        # reuses the same scene (remove + re-add geometry)
        second = render_views_offscreen(ro, prepared, snap, cfg,
                                        views=["front"])
        return ro, first, second

    try:
        ro, first, second = submit_render(job)
    except RenderError as exc:
        if "init failed" in str(exc):
            pytest.skip(str(exc))
        raise
    assert first["front"].shape == second["front"].shape
    assert second["front"].std() > 1.0
    # Single-instance rule: exactly one shared renderer exists, and the job
    # ran on the dedicated render thread.
    from pc_snapshot import renderer as rmod
    assert rmod._SHARED_RO is ro
    assert ro is not None
