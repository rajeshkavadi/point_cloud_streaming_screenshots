"""Shared test fixtures: a synthetic PointCloud2-like message builder.

Works without ROS by producing a duck-typed object with the same attribute
names sensor_msgs/PointCloud2 exposes.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import List

import numpy as np


@dataclass
class FakeField:
    name: str
    offset: int
    datatype: int
    count: int = 1


@dataclass
class FakeHeaderStamp:
    sec: int = 0
    nanosec: int = 0


@dataclass
class FakeHeader:
    stamp: FakeHeaderStamp = field(default_factory=FakeHeaderStamp)
    frame_id: str = "camera_init"


@dataclass
class FakePointCloud2:
    header: FakeHeader
    height: int
    width: int
    fields: List[FakeField]
    is_bigendian: bool
    point_step: int
    row_step: int
    data: bytes
    is_dense: bool = True


FLOAT32 = 7
FLOAT64 = 8


def make_xyz_cloud(n: int = 1000, frame_id: str = "camera_init",
                   big_endian: bool = False, row_padding: int = 0,
                   include_intensity: bool = True,
                   inject_nonfinite: bool = False) -> FakePointCloud2:
    """Deterministic synthetic XYZ(+intensity) cloud in a 20x20x5 box."""
    pts = []
    for i in range(n):
        x = (i % 100) * 0.2 - 10.0
        y = ((i // 100) % 100) * 0.2 - 10.0
        z = (i // 10000) * 0.25
        inten = float(i % 255)
        pts.append((x, y, z, inten))
    if inject_nonfinite and pts:
        pts[0] = (float("nan"), 0.0, 0.0, 0.0)
        if len(pts) > 1:
            pts[1] = (float("inf"), 1.0, 1.0, 1.0)

    endian = ">" if big_endian else "<"
    if include_intensity:
        fields = [
            FakeField("x", 0, FLOAT32),
            FakeField("y", 4, FLOAT32),
            FakeField("z", 8, FLOAT32),
            FakeField("intensity", 12, FLOAT32),
        ]
        point_step = 16
        packer = lambda p: struct.pack(endian + "ffff", *p)
    else:
        fields = [
            FakeField("x", 0, FLOAT32),
            FakeField("y", 4, FLOAT32),
            FakeField("z", 8, FLOAT32),
        ]
        point_step = 12
        packer = lambda p: struct.pack(endian + "fff", p[0], p[1], p[2])

    row_bytes = point_step * n
    row_step = row_bytes + row_padding
    buf = b"".join(packer(p) for p in pts)
    if row_padding:
        buf += b"\x00" * row_padding

    return FakePointCloud2(
        header=FakeHeader(stamp=FakeHeaderStamp(sec=100, nanosec=500000000),
                          frame_id=frame_id),
        height=1,
        width=n,
        fields=fields,
        is_bigendian=big_endian,
        point_step=point_step,
        row_step=row_step,
        data=buf,
    )


def make_scene_cloud(n_ground: int = 4000) -> FakePointCloud2:
    """Scene with ground, walls and a box -> good for visual inspection."""
    pts = []
    side = int(math.sqrt(n_ground))
    for i in range(side):
        for j in range(side):
            x = i * 0.5 - side * 0.25
            y = j * 0.5 - side * 0.25
            z = 0.05 * math.sin(0.4 * x) * math.cos(0.4 * y)
            pts.append((x, y, z, 40.0 + 3 * z))
    # wall
    for k in range(40):
        pts.append((6.0, -6.0 + 0.3 * k, 0.1 * k, 150.0))
    # box
    for k in range(20):
        for m in (-1.0, 1.0):
            pts.append((2.0 + m, 2.0, 0.1 * k, 220.0))
            pts.append((2.0, 2.0 + m, 0.1 * k, 220.0))
    n = len(pts)
    fields = [
        FakeField("x", 0, FLOAT32),
        FakeField("y", 4, FLOAT32),
        FakeField("z", 8, FLOAT32),
        FakeField("intensity", 12, FLOAT32),
    ]
    buf = b"".join(struct.pack("<ffff", *p) for p in pts)
    return FakePointCloud2(
        header=FakeHeader(frame_id="camera_init"),
        height=1, width=n, fields=fields, is_bigendian=False,
        point_step=16, row_step=16 * n, data=buf,
    )


def snapshot_from_cloud(msg: FakePointCloud2, seq: int = 1):
    """Decode a fake message into a PointCloudSnapshot (test helper)."""
    import time
    from pc_snapshot.decoder import decode_pointcloud2
    from pc_snapshot.store import PointCloudSnapshot

    dec = decode_pointcloud2(msg)
    return PointCloudSnapshot(
        points=dec.points,
        intensity=dec.intensity,
        frame_id=msg.header.frame_id,
        ros_stamp=100.5,
        received_walltime=time.time(),
        topic="/test/points",
        total_points_in_message=dec.total_points,
        seq=seq,
    )
