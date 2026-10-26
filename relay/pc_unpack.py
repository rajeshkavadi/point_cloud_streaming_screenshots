"""Generic sensor_msgs/PointCloud2 -> numpy unpacker.

Handles arbitrary field layouts (offsets, datatypes, endianness, point_step),
non-contiguous XYZ fields, and non-finite points. No ROS imports so it can be
unit-tested without a ROS environment.
"""

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

_DATATYPE_MAP = {
    INT8: np.int8,
    UINT8: np.uint8,
    INT16: np.int16,
    UINT16: np.uint16,
    INT32: np.int32,
    UINT32: np.uint32,
    FLOAT32: np.float32,
    FLOAT64: np.float64,
}

XYZ_NAMES = ("x", "y", "z")
INTENSITY_NAMES = ("intensity", "i")


def fields_to_dtype(fields, point_step, is_bigendian=False):
    """Build a numpy structured dtype from PointField descriptors.

    fields: iterable of (name, offset, datatype) or (name, offset, datatype, count)
    """
    endian = ">" if is_bigendian else "<"
    np_fields = []
    for field in fields:
        name, offset, datatype = field[0], field[1], field[2]
        count = field[3] if len(field) > 3 else 1
        base = _DATATYPE_MAP.get(int(datatype))
        if base is None:
            continue  # skip unknown datatypes
        base = np.dtype(endian + base().dtype.char)
        if count and int(count) > 1:
            np_fields.append((name, (base, (int(count),)), int(offset)))
        else:
            np_fields.append((name, base, int(offset)))
    return np.dtype({"names": [f[0] for f in np_fields],
                     "formats": [f[1] for f in np_fields],
                     "offsets": [f[2] for f in np_fields],
                     "itemsize": int(point_step)})


def unpack_xyz(data, fields, point_step, is_bigendian=False):
    """Unpack raw PointCloud2 data into x, y, z float32 arrays (no filtering).

    Returns (x, y, z) as numpy float32 arrays, one entry per point.
    Raises KeyError if x/y/z fields are absent.
    """
    dtype = fields_to_dtype(fields, point_step, is_bigendian)
    arr = np.frombuffer(bytes(data), dtype=dtype)
    x = np.asarray(arr["x"], dtype=np.float32)
    y = np.asarray(arr["y"], dtype=np.float32)
    z = np.asarray(arr["z"], dtype=np.float32)
    return x, y, z


def unpack_xyz_intensity(data, fields, point_step, is_bigendian=False):
    """Unpack into (x, y, z, intensity) float32 arrays.

    intensity is None if no intensity-like field exists.
    """
    x, y, z = unpack_xyz(data, fields, point_step, is_bigendian)
    dtype = fields_to_dtype(fields, point_step, is_bigendian)
    arr = np.frombuffer(bytes(data), dtype=dtype)
    names = set(dtype.names or ())
    intensity = None
    for cand in INTENSITY_NAMES:
        if cand in names:
            intensity = np.asarray(arr[cand], dtype=np.float32)
            break
    return x, y, z, intensity


def finite_mask(x, y, z):
    """Boolean mask of points where x, y and z are all finite."""
    return np.isfinite(x) & np.isfinite(y) & np.isfinite(z)


def clip_mask(x, y, z, clip_range=None, z_min=None, z_max=None):
    """Boolean mask applying radial range and z bounds (None = unbounded)."""
    mask = np.ones(x.shape, dtype=bool)
    if clip_range is not None:
        r2 = x * x + y * y + z * z
        mask &= r2 <= float(clip_range) ** 2
    if z_min is not None:
        mask &= z >= float(z_min)
    if z_max is not None:
        mask &= z <= float(z_max)
    return mask


def downsample_indices(n, max_points):
    """Indices that reduce n points to at most max_points (uniform stride)."""
    if max_points is None or n <= max_points:
        return None
    stride = int(np.ceil(n / max_points))
    return np.arange(0, n, stride)


def process_frame(x, y, z, intensity=None, max_points=None,
                  clip_range=None, z_min=None, z_max=None):
    """Full pipeline: filter invalid, clip, downsample.

    Returns (x, y, z, intensity, stats) with float32 arrays of equal length.
    stats = {"input": n_in, "output": n_out, "filtered_invalid": k,
             "filtered_clip": m}
    """
    n_in = x.shape[0]
    mask = finite_mask(x, y, z)
    n_invalid = int(n_in - np.count_nonzero(mask))
    if clip_range is not None or z_min is not None or z_max is not None:
        clip_m = clip_mask(x, y, z, clip_range, z_min, z_max)
        mask &= clip_m
    n_after_clip = int(np.count_nonzero(mask))
    idx = np.flatnonzero(mask)
    if idx.size:
        ds = downsample_indices(idx.size, max_points)
        if ds is not None:
            idx = idx[ds]
    xi = np.ascontiguousarray(x[idx], dtype=np.float32)
    yi = np.ascontiguousarray(y[idx], dtype=np.float32)
    zi = np.ascontiguousarray(z[idx], dtype=np.float32)
    ii = np.ascontiguousarray(intensity[idx], dtype=np.float32) if intensity is not None else None
    stats = {"input": int(n_in), "output": int(idx.size),
             "filtered_invalid": n_invalid,
             "filtered_clip": n_after_clip and int(n_after_clip - idx.size)}
    return xi, yi, zi, ii, stats
