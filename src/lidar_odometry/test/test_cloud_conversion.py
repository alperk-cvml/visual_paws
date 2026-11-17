"""Unit tests for PointCloud2 → numpy conversion.

`normalize_scan_times` is pure numpy and always runs; the PointCloud2
extraction tests are skipped automatically outside a ROS environment.
"""

import numpy as np
import pytest

from lidar_odometry.cloud_conversion import normalize_scan_times


def test_normalize_relative_seconds():
    t = normalize_scan_times([0.0, 0.05, 0.1])
    assert np.allclose(t, [0.0, 0.5, 1.0])


def test_normalize_absolute_nanoseconds():
    base = 1.7e18
    t = normalize_scan_times(np.array([base, base + 5e7, base + 1e8]))
    assert np.allclose(t, [0.0, 0.5, 1.0])


def test_normalize_degenerate_inputs():
    assert normalize_scan_times([]).size == 0
    assert np.allclose(normalize_scan_times([3.0, 3.0, 3.0]), 0.0)
    assert np.allclose(normalize_scan_times([np.nan, np.nan]), 0.0)


def _make_cloud(points, times=None):
    from sensor_msgs.msg import PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    header = Header()
    header.frame_id = 'lidar_link'
    fields = [
        PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate(('x', 'y', 'z'))
    ]
    if times is None:
        return point_cloud2.create_cloud(header, fields, points)
    fields.append(PointField(name='time', offset=12,
                             datatype=PointField.FLOAT32, count=1))
    rows = [(*p, t) for p, t in zip(points, times)]
    return point_cloud2.create_cloud(header, fields, rows)


def test_extract_xyz_filters_non_finite():
    pytest.importorskip('sensor_msgs_py')
    from lidar_odometry.cloud_conversion import extract_xyz_and_times

    cloud = _make_cloud([(1.0, 2.0, 3.0),
                         (float('nan'), 0.0, 0.0),
                         (4.0, 5.0, 6.0)])
    xyz, times = extract_xyz_and_times(cloud)
    assert xyz.shape == (2, 3)
    assert xyz.dtype == np.float64
    assert np.allclose(xyz, [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
    assert times is None


def test_extract_times_stay_row_aligned_with_xyz():
    pytest.importorskip('sensor_msgs_py')
    from lidar_odometry.cloud_conversion import extract_xyz_and_times

    cloud = _make_cloud(
        [(1.0, 0.0, 0.0), (float('nan'), 0.0, 0.0), (2.0, 0.0, 0.0)],
        times=[0.0, 0.05, 0.1])
    xyz, times = extract_xyz_and_times(cloud)
    assert xyz.shape == (2, 3)
    # NaN row removed from both arrays: remaining times are the ends of scan.
    assert np.allclose(times, [0.0, 1.0])
