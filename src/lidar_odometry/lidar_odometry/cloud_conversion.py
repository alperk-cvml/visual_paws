"""PointCloud2 → numpy conversion for the LiDAR odometry pipeline.

KISS-ICP wants an Nx3 float64 array plus, when motion compensation (deskew)
is on, one per-point timestamp normalized to [0, 1] across the scan. Real
drivers disagree on the time field's name, unit, and epoch (Ouster: `t` in
nanoseconds, Velodyne: `time` in relative seconds, Livox: `timestamp` in
absolute nanoseconds, …), so timestamps are min-max normalized rather than
interpreted — deskewing only needs the relative position within the sweep.
"""

import numpy as np

# Checked in order; first match wins.
TIME_FIELD_CANDIDATES = ('t', 'time', 'timestamp', 'time_offset',
                         'point_time_offset', 'stamps')


def normalize_scan_times(raw) -> np.ndarray:
    """Min-max normalize per-point times to [0, 1], unit/epoch agnostic.

    Degenerate inputs (empty, constant, or non-finite) come back as zeros,
    which makes deskewing a no-op instead of an error.
    """
    t = np.asarray(raw, dtype=np.float64).ravel()
    if t.size == 0:
        return t
    finite = np.isfinite(t)
    if not finite.any():
        return np.zeros_like(t)
    lo = t[finite].min()
    hi = t[finite].max()
    if hi <= lo:
        return np.zeros_like(t)
    out = np.where(finite, (t - lo) / (hi - lo), 0.0)
    return out


def extract_xyz_and_times(msg):
    """PointCloud2 → (Nx3 float64 finite xyz, normalized times or None).

    The time column is read separately from xyz (dtypes differ) and filtered
    with the same finite-point mask so the two stay row-aligned.
    """
    from sensor_msgs_py import point_cloud2

    xyz = point_cloud2.read_points_numpy(
        msg, field_names=('x', 'y', 'z'), skip_nans=False).astype(np.float64)
    finite = np.isfinite(xyz).all(axis=1)

    times = None
    field_names = {f.name for f in msg.fields}
    for candidate in TIME_FIELD_CANDIDATES:
        if candidate in field_names:
            raw = point_cloud2.read_points(
                msg, field_names=(candidate,), skip_nans=False)[candidate]
            times = normalize_scan_times(raw)
            break

    xyz = xyz[finite]
    if times is not None:
        times = times[finite]
    return xyz, times
