"""Geometric subtraction of one point cloud from another.

Subtracting cloud ``A`` from cloud ``B`` keeps every point of ``B`` that has no
nearby counterpart in ``A``: for each point in ``B`` the distance to its
nearest neighbour in ``A`` is measured, and the point survives only when that
distance exceeds a threshold. This is what isolates the food on a plate --
``B`` is the captured scene and ``A`` is the (registered) stored model of the
empty plate, so what remains is everything the model does not explain.

The threshold is not defaulted here: it is the node's
``food_subtraction_distance_threshold`` parameter, held by
:class:`grasp_pose_provider.node_parameters.GraspPoseProviderParameters` and
handed down through
:func:`grasp_pose_provider.grasp_candidate_generation.food_detector.detect_food`.

The nearest distances come from
:meth:`open3d.geometry.PointCloud.compute_point_cloud_distance`, which builds a
KD-tree over ``A`` internally and is far cheaper than a naive all-pairs search.
"""

import numpy as np
import open3d as o3d


def subtract_indices(cloud_a, cloud_b, distance_threshold):
    """Return the indices of ``cloud_b`` points that survive the subtraction.

    A point of ``cloud_b`` survives when its nearest neighbour in ``cloud_a``
    is farther than ``distance_threshold`` metres. The returned array indexes
    into ``cloud_b``.
    """
    if len(cloud_a.points) == 0:
        return np.arange(len(cloud_b.points))

    distances = np.asarray(cloud_b.compute_point_cloud_distance(cloud_a))
    return np.where(distances > distance_threshold)[0]


def subtract(cloud_a, cloud_b, distance_threshold):
    if len(cloud_a.points) == 0:
        return o3d.geometry.PointCloud(cloud_b)

    kept_indices = subtract_indices(cloud_a, cloud_b, distance_threshold)
    return cloud_b.select_by_index(kept_indices.tolist())
