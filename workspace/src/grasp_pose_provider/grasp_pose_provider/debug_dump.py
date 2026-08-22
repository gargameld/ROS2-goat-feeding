"""Temporary debug dump of the food-detection intermediates.

Writes the registration transform, the stored and captured clouds, the food
cloud on its own, and the food indices straight into
``src/grasp_pose_provider/debug_pointclouds`` so the object detection can be
inspected offline. Because it writes into the source tree directly,
``setup.py`` does not have to install the folder.

This is an intermediate debugging aid only; remove this module (and its call
site in :mod:`grasp_pose_provider.food_detector`) once detection is trusted.
"""

import os

import numpy as np
import open3d as o3d


# This file lives at src/grasp_pose_provider/grasp_pose_provider/debug_dump.py;
# the debug folder sits one level up, next to the package's python module dir.
# ``realpath`` resolves the symlink created by ``colcon build
# --symlink-install`` back to this source file, so the dump lands in the source
# tree rather than the install space.
DEBUG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'debug_pointclouds',
)


def dump_detection(
    stored_cloud,
    captured_cloud,
    transform,
    food_indices,
    directory=DEBUG_DIR,
):
    """Write the detection intermediates to the debug folder (best-effort).

    ``food_indices`` index into ``captured_cloud`` (the Open3D cloud dumped
    here), so the food points can be highlighted directly on the dumped cloud.
    Any failure is swallowed on purpose: this debugging aid must never break
    the detection pipeline.
    """
    try:
        os.makedirs(directory, exist_ok=True)
        o3d.io.write_point_cloud(
            os.path.join(directory, 'stored_cloud.pcd'), stored_cloud
        )
        o3d.io.write_point_cloud(
            os.path.join(directory, 'captured_cloud.pcd'), captured_cloud
        )
        # The food points on their own, so the segmentation can be eyeballed
        # without having to re-apply the indices to the captured cloud.
        o3d.io.write_point_cloud(
            os.path.join(directory, 'food_cloud.pcd'),
            captured_cloud.select_by_index(
                np.asarray(food_indices, dtype=np.int64).tolist()
            ),
        )
        np.savetxt(
            os.path.join(directory, 'transform.txt'), np.asarray(transform)
        )
        np.savetxt(
            os.path.join(directory, 'food_indices.txt'),
            np.asarray(food_indices, dtype=np.int64),
            fmt='%d',
        )
    except Exception:  # noqa: BLE001 - debugging aid must be non-fatal
        pass
