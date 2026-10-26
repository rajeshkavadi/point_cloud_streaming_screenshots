"""Generic sensor_msgs/PointCloud2 -> numpy decoder.

Interprets arbitrary field layouts: respects field offsets, point_step,
row_step, height/width (including row padding) and message endianness.
Supports XYZ plus optional intensity-like fields. Filters non-finite points.

No ROS imports: works on any duck-typed object exposing the PointCloud2
attributes (fields with name/offset/datatype/count, data, point_step,
row_step, width, height, is_big_endian), which allows unit testing without a
ROS environment.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

# sensor_msgs/PointField datatype codes
INT8 = 1
UINT8 = 2
INT16 = 3
UINT16 = 4
INT32 = 5
UINT32 = 6
FLOAT32 = 7
FLOAT64 = 8

_DATATYPE_NP = {
    INT8: ("i1", 1),
    UINT8: ("u1", 1),
    INT16: ("i2", 2),
    UINT16: ("u2", 2),
    INT32: ("i4", 4),
    UINT32: ("u4", 4),
    FLOAT32: ("f4", 4),
    FLOAT64: ("f8", 8),
}

# Candidate field names for intensity, in order of preference.
INTENSITY_FIELD_NAMES = ("intensity", "reflectivity", "i", "ring_intensity")


class PointCloudDecodeError(Exception):
    """Raised when a PointCloud2 message cannot be decoded."""


@dataclass(frozen=True)
class DecodedCloud:
    """Decoded point cloud.

    points: (N, 3) float32 array of X, Y, Z (non-finite points removed).
    intensity: (N,) float32 array or None when no intensity field exists.
    """

    points: np.ndarray
    intensity: Optional[np.ndarray]
    total_points: int  # points before the non-finite filter
    source_fields: Tuple[str, ...]  # field names present in the message


def _field_dtype_code(datatype: int) -> str:
    try:
        return _DATATYPE_NP[datatype][0]
    except KeyError:
        raise PointCloudDecodeError(f"unsupported PointField datatype code: {datatype}")


def _find_field(fields: Sequence, name: str) -> Optional[object]:
    for f in fields:
        if f.name == name:
            return f
    return None


def _structured_dtype(fields: Sequence, names: Sequence[str], point_step: int,
                      big_endian: bool) -> np.dtype:
    """Build a structured dtype selecting named scalar fields at their offsets."""
    endian = ">" if big_endian else "<"
    offsets = []
    formats = []
    for name in names:
        f = _find_field(fields, name)
        fmt = _field_dtype_code(f.datatype)
        size = _DATATYPE_NP[f.datatype][1]
        off = int(f.offset)
        if off + size > point_step:
            raise PointCloudDecodeError(
                f"field '{name}' (offset {off}, size {size}) exceeds point_step {point_step}")
        offsets.append(off)
        formats.append(endian + fmt)
    return np.dtype({"names": list(names), "formats": formats,
                     "offsets": offsets, "itemsize": point_step})


def _row_major_view(data: bytes, width: int, height: int, point_step: int,
                    row_step: int) -> np.ndarray:
    """Return a (width*height, point_step) uint8 view honoring row padding."""
    raw = np.frombuffer(data, dtype=np.uint8)
    n = width * height
    if n == 0:
        return raw[:0]
    needed = n * point_step
    row_bytes = width * point_step
    if row_step == row_bytes:
        if raw.size < needed:
            raise PointCloudDecodeError(
                f"data buffer too small: have {raw.size} bytes, need {needed}")
        return raw[:needed]
    # Row padding present: gather the used portion of every row.
    if raw.size < height * row_step:
        raise PointCloudDecodeError(
            f"data buffer too small for rows: have {raw.size} bytes, "
            f"need {height * row_step}")
    rows = raw[:height * row_step].reshape(height, row_step)[:, :row_bytes]
    return rows.reshape(-1)


def decode_pointcloud2(msg) -> DecodedCloud:
    """Decode a PointCloud2-like message into numpy arrays.

    Raises PointCloudDecodeError on malformed messages.
    """
    fields = getattr(msg, "fields", None)
    if fields is None or len(fields) == 0:
        raise PointCloudDecodeError("message has no fields")
    point_step = int(getattr(msg, "point_step", 0))
    if point_step <= 0:
        raise PointCloudDecodeError(f"invalid point_step: {point_step}")
    width = int(getattr(msg, "width", 0))
    height = int(getattr(msg, "height", 1))
    if height == 0:
        height = 1
    row_step = int(getattr(msg, "row_step", 0)) or width * point_step
    # ROS 2 names the field `is_bigendian`; accept a few variants.
    big_endian = bool(getattr(msg, "is_bigendian",
                              getattr(msg, "is_big_endian", False)))
    data = getattr(msg, "data", b"")
    if isinstance(data, (list, tuple)):
        data = bytes(data)
    elif not isinstance(data, (bytes, bytearray, memoryview)):
        data = bytes(data)
    data = bytes(data)

    for axis in ("x", "y", "z"):
        if _find_field(fields, axis) is None:
            raise PointCloudDecodeError(f"message is missing required field '{axis}'")

    raw = _row_major_view(data, width, height, point_step, row_step)
    n_total = width * height

    xyz_dtype = _structured_dtype(fields, ("x", "y", "z"), point_step, big_endian)
    try:
        structured = raw.view(xyz_dtype)
    except Exception as exc:  # pragma: no cover - defensive
        raise PointCloudDecodeError(f"cannot interpret point buffer: {exc}")
    if structured.shape[0] < n_total:
        raise PointCloudDecodeError(
            f"buffer holds {structured.shape[0]} points, header claims {n_total}")
    structured = structured[:n_total]

    xyz = np.empty((n_total, 3), dtype=np.float32)
    xyz[:, 0] = structured["x"]
    xyz[:, 1] = structured["y"]
    xyz[:, 2] = structured["z"]

    # Optional intensity-like field.
    intensity = None
    int_field_name = None
    for cand in INTENSITY_FIELD_NAMES:
        f = _find_field(fields, cand)
        if f is not None:
            int_field_name = cand
            break
    if int_field_name is not None:
        try:
            int_dtype = _structured_dtype(fields, (int_field_name,), point_step, big_endian)
            int_struct = raw.view(int_dtype)[:n_total]
            intensity = np.empty(n_total, dtype=np.float32)
            intensity[:] = int_struct[int_field_name]
        except PointCloudDecodeError:
            intensity = None  # degrade gracefully; XYZ still usable

    finite = np.isfinite(xyz).all(axis=1)
    points = np.ascontiguousarray(xyz[finite], dtype=np.float32)
    int_out = None
    if intensity is not None:
        int_out = np.ascontiguousarray(intensity[finite], dtype=np.float32)

    if points.flags.writeable:
        points.flags.writeable = False
    if int_out is not None and int_out.flags.writeable:
        int_out.flags.writeable = False

    return DecodedCloud(points=points, intensity=int_out, total_points=n_total,
                        source_fields=tuple(f.name for f in fields))


def downsample_points(points: np.ndarray, intensity: Optional[np.ndarray],
                      max_points: int) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Deterministic stride downsample to at most max_points points.

    Returns (points, intensity); intensity may be None. Input is never
    modified; outputs are new arrays (writable copies only if needed).
    """
    n = points.shape[0]
    if max_points <= 0 or n <= max_points:
        return points, intensity
    stride = int(np.ceil(n / max_points))
    idx = np.arange(0, n, stride)
    pts = np.array(points[idx], dtype=np.float32, copy=True)
    inten = None
    if intensity is not None:
        inten = np.array(intensity[idx], dtype=np.float32, copy=True)
    return pts, inten
