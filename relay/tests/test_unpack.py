"""Unit tests for relay/pc_unpack.py — PointCloud2 decoding edge cases.

Run:  python3 -m unittest discover -s relay/tests -v
No ROS required.
"""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pc_unpack import (  # noqa: E402
    unpack_xyz, unpack_xyz_intensity, process_frame, downsample_indices,
    INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64,
)


def encode(points, layout, point_step, is_bigendian=False):
    """Encode [(x,y,z,...)] tuples into raw bytes per a layout spec.

    layout: list of (name, offset, datatype). Values taken from point tuple in
    layout order.
    """
    endian = ">" if is_bigendian else "<"
    dtmap = {INT8: "i1", UINT8: "u1", INT16: "i2", UINT16: "u2",
             INT32: "i4", UINT32: "u4", FLOAT32: "f4", FLOAT64: "f8"}
    buf = bytearray(point_step * len(points))
    for i, pt in enumerate(points):
        for (name, offset, datatype), val in zip(layout, pt):
            raw = np.array([val], dtype=np.dtype(endian + dtmap[datatype])).tobytes()
            base = i * point_step + offset
            buf[base:base + len(raw)] = raw
    return bytes(buf)


XYZ = [("x", 0, FLOAT32), ("y", 4, FLOAT32), ("z", 8, FLOAT32)]


class TestContiguousXYZ(unittest.TestCase):
    def test_basic(self):
        pts = [(1.0, 2.0, 3.0), (-4.5, 0.0, 10.25)]
        data = encode(pts, XYZ, 12)
        x, y, z = unpack_xyz(data, XYZ, 12)
        np.testing.assert_allclose(x, [1.0, -4.5])
        np.testing.assert_allclose(y, [2.0, 0.0])
        np.testing.assert_allclose(z, [3.0, 10.25])


class TestNonContiguousFields(unittest.TestCase):
    def test_padded_layout(self):
        # XYZ interleaved with a tag field and padding (like many real drivers)
        layout = [("x", 0, FLOAT32), ("tag", 4, UINT16), ("pad", 6, UINT16),
                  ("y", 8, FLOAT32), ("z", 16, FLOAT32)]
        pts = [(1.0, 7, 0, 2.0, 3.0), (4.0, 9, 0, 5.0, 6.0)]
        data = encode(pts, layout, 24)
        x, y, z = unpack_xyz(data, layout, 24)
        np.testing.assert_allclose(x, [1.0, 4.0])
        np.testing.assert_allclose(y, [2.0, 5.0])
        np.testing.assert_allclose(z, [3.0, 6.0])

    def test_bigendian(self):
        data = encode([(1.0, 2.0, 3.0)], XYZ, 12, is_bigendian=True)
        x, y, z = unpack_xyz(data, XYZ, 12, is_bigendian=True)
        np.testing.assert_allclose([x[0], y[0], z[0]], [1.0, 2.0, 3.0])


class TestDatatypes(unittest.TestCase):
    def test_float64_xyz(self):
        layout = [("x", 0, FLOAT64), ("y", 8, FLOAT64), ("z", 16, FLOAT64)]
        data = encode([(1.5, -2.5, 3.5)], layout, 24)
        x, y, z = unpack_xyz(data, layout, 24)
        np.testing.assert_allclose([x[0], y[0], z[0]], [1.5, -2.5, 3.5])

    def test_int16_xyz(self):
        layout = [("x", 0, INT16), ("y", 2, INT16), ("z", 4, INT16)]
        data = encode([(100, -200, 300)], layout, 6)
        x, y, z = unpack_xyz(data, layout, 6)
        np.testing.assert_allclose([x[0], y[0], z[0]], [100, -200, 300])

    def test_intensity_extraction(self):
        layout = XYZ + [("intensity", 12, FLOAT32)]
        data = encode([(1.0, 2.0, 3.0, 0.75)], layout, 16)
        x, y, z, i = unpack_xyz_intensity(data, layout, 16)
        self.assertIsNotNone(i)
        np.testing.assert_allclose(i, [0.75])

    def test_missing_intensity_returns_none(self):
        data = encode([(1.0, 2.0, 3.0)], XYZ, 12)
        _, _, _, i = unpack_xyz_intensity(data, XYZ, 12)
        self.assertIsNone(i)


class TestInvalidPoints(unittest.TestCase):
    def test_nan_inf_filtered(self):
        nan = float("nan")
        inf = float("inf")
        pts = [(1.0, 2.0, 3.0), (nan, 2.0, 3.0), (1.0, inf, 3.0),
               (1.0, 2.0, -inf), (4.0, 5.0, 6.0)]
        data = encode(pts, XYZ, 12)
        x, y, z, i, stats = process_frame(*unpack_xyz(data, XYZ, 12))
        self.assertEqual(x.size, 2)
        np.testing.assert_allclose(x, [1.0, 4.0])
        self.assertEqual(stats["filtered_invalid"], 3)
        self.assertEqual(stats["input"], 5)
        self.assertEqual(stats["output"], 2)


class TestDownsample(unittest.TestCase):
    def test_stride(self):
        self.assertIsNone(downsample_indices(100, 1000))
        idx = downsample_indices(1000, 100)
        self.assertLessEqual(idx.size, 100)
        self.assertGreaterEqual(idx.size, 99)

    def test_cap(self):
        rng = np.random.default_rng(0)
        n = 123456
        x = rng.standard_normal(n).astype(np.float32)
        y = rng.standard_normal(n).astype(np.float32)
        z = rng.standard_normal(n).astype(np.float32)
        xr, yr, zr, _, stats = process_frame(x, y, z, max_points=50000)
        self.assertLessEqual(xr.size, 50000)
        self.assertEqual(stats["input"], n)


class TestClip(unittest.TestCase):
    def test_range_clip(self):
        pts = [(1.0, 0.0, 0.0), (500.0, 0.0, 0.0), (3.0, 4.0, 0.0)]
        data = encode(pts, XYZ, 12)
        x, y, z, _, stats = process_frame(*unpack_xyz(data, XYZ, 12), clip_range=10.0)
        self.assertEqual(x.size, 2)
        np.testing.assert_allclose(x, [1.0, 3.0])

    def test_z_clip(self):
        pts = [(0.0, 0.0, -5.0), (0.0, 0.0, 0.5), (0.0, 0.0, 30.0)]
        data = encode(pts, XYZ, 12)
        x, y, z, _, _ = process_frame(*unpack_xyz(data, XYZ, 12), z_min=-1.0, z_max=10.0)
        self.assertEqual(z.size, 1)
        np.testing.assert_allclose(z, [0.5])


class TestEmpty(unittest.TestCase):
    def test_empty_cloud(self):
        x, y, z, i, stats = process_frame(np.array([], dtype=np.float32),
                                          np.array([], dtype=np.float32),
                                          np.array([], dtype=np.float32))
        self.assertEqual(x.size, 0)
        self.assertEqual(stats["input"], 0)


if __name__ == "__main__":
    unittest.main()
