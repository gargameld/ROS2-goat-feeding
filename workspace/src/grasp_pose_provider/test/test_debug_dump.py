"""Tests for pre- and post-GPD debug dump names."""

import numpy as np
import open3d as o3d

from grasp_pose_provider import debug_dump


def test_after_gpd_dump_has_distinct_food_cloud(tmp_path):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector([[1.0, 2.0, 3.0]])

    debug_dump.dump_detection(
        cloud,
        cloud,
        np.identity(4),
        [0],
        directory=str(tmp_path),
        stage='after_gpd',
    )

    assert (tmp_path / 'food_cloud_after_gpd.pcd').is_file()
    assert (tmp_path / 'captured_cloud_after_gpd.pcd').is_file()
    assert (tmp_path / 'food_indices_after_gpd.txt').is_file()
