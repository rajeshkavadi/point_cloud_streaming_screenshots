"""Unit tests for the PointCloud2 decoder (no ROS required)."""

import struct

import numpy as np
import pytest

from pc_snapshot.decoder import (PointCloudDecodeError, decode_pointcloud2,
                                 downsample_points)
from tests.fixtures import FLOAT64, FakeField, make_xyz_cloud


def test_basic_xyz_intensity():
    msg = make_xyz_cloud(1000, include_intensity=True)
    dec = decode_pointcloud2(msg)
    assert dec.points.shape == (1000, 3)
    assert dec.intensity is not None and dec.intensity.shape == (1000,)
    # deterministic first point
    np.testing.assert_allclose(dec.points[0], [-10.0, -10.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(dec.intensity[0], 0.0)
    assert dec.total_points == 1000
    assert "intensity" in dec.source_fields


def test_xyz_only():
    msg = make_xyz_cloud(500, include_intensity=False)
    dec = decode_pointcloud2(msg)
    assert dec.points.shape == (500, 3)
    assert dec.intensity is None


def test_big_endian():
    msg = make_xyz_cloud(256, big_endian=True)
    dec = decode_pointcloud2(msg)
    assert dec.points.shape == (256, 3)
    np.testing.assert_allclose(dec.points[0], [-10.0, -10.0, 0.0], atol=1e-6)


def test_row_padding():
    msg = make_xyz_cloud(300, row_padding=12)
    dec = decode_pointcloud2(msg)
    assert dec.points.shape == (300, 3)
    np.testing.assert_allclose(dec.points[10], decode_pointcloud2(
        make_xyz_cloud(300)).points[10], atol=1e-6)


def test_nonfinite_filtered():
    msg = make_xyz_cloud(100, inject_nonfinite=True)
    dec = decode_pointcloud2(msg)
    assert dec.total_points == 100
    assert dec.points.shape[0] == 98  # nan + inf removed
    assert np.isfinite(dec.points).all()


def test_empty_cloud():
    msg = make_xyz_cloud(0)
    dec = decode_pointcloud2(msg)
    assert dec.points.shape == (0, 3)


def test_nonstandard_offsets_and_mixed_datatypes():
    """Fields at unusual offsets with a FLOAT64 z must still decode."""
    pts = [(1.0, 2.0, 3.0, 0.0), (-4.5, 0.25, 9.75, 1.0), (10.0, -3.0, 0.0, 2.0)]
    fields = [
        FakeField("intensity", 0, FLOAT64),
        FakeField("x", 8, FLOAT64),
        FakeField("y", 16, FLOAT64),
        FakeField("z", 24, FLOAT64),
    ]
    buf = b"".join(struct.pack("<dddd", p[3], p[0], p[1], p[2]) for p in pts)
    from tests.fixtures import FakePointCloud2, FakeHeader
    msg = FakePointCloud2(header=FakeHeader(), height=1, width=3, fields=fields,
                          is_bigendian=False, point_step=32, row_step=96, data=buf)
    dec = decode_pointcloud2(msg)
    assert dec.points.shape == (3, 3)
    np.testing.assert_allclose(dec.points[1], [-4.5, 0.25, 9.75], atol=1e-6)
    np.testing.assert_allclose(dec.intensity, [0.0, 1.0, 2.0], atol=1e-6)


def test_missing_xyz_raises():
    msg = make_xyz_cloud(10)
    msg.fields = [f for f in msg.fields if f.name != "z"]
    with pytest.raises(PointCloudDecodeError):
        decode_pointcloud2(msg)


def test_no_fields_raises():
    msg = make_xyz_cloud(10)
    msg.fields = []
    with pytest.raises(PointCloudDecodeError):
        decode_pointcloud2(msg)


def test_truncated_buffer_raises():
    msg = make_xyz_cloud(100)
    msg.data = msg.data[: 50 * msg.point_step]
    msg.width = 100
    with pytest.raises(PointCloudDecodeError):
        decode_pointcloud2(msg)


def test_downsample_deterministic():
    msg = make_xyz_cloud(10_000)
    dec = decode_pointcloud2(msg)
    a, ai = downsample_points(dec.points, dec.intensity, 1000)
    b, bi = downsample_points(dec.points, dec.intensity, 1000)
    assert a.shape[0] <= 1000
    np.testing.assert_array_equal(a, b)
    np.testing.assert_array_equal(ai, bi)
    # input untouched
    assert dec.points.shape[0] == 10_000


def test_downsample_noop_when_small():
    msg = make_xyz_cloud(100)
    dec = decode_pointcloud2(msg)
    a, _ = downsample_points(dec.points, dec.intensity, 1000)
    assert a.shape[0] == 100


def test_snapshot_arrays_readonly():
    from pc_snapshot.store import PointCloudSnapshot
    import time
    dec = decode_pointcloud2(make_xyz_cloud(50))
    snap = PointCloudSnapshot(points=dec.points, intensity=dec.intensity,
                              frame_id="f", ros_stamp=0.0,
                              received_walltime=time.time(),
                              topic="/t", total_points_in_message=50, seq=1)
    with pytest.raises(ValueError):
        snap.points[0, 0] = 1.0
