"""Reject GPD candidates that do not describe a useful food grasp."""

import math

import numpy as np
import open3d as o3d


DEFAULT_MIN_SCORE = 0.0
DEFAULT_MAX_POSITION_DISTANCE = 0.05
DEFAULT_AXIS_ALIGNMENT_DEG = 20.0


def filter_grasps(
    grasp_config_list,
    food_points,
    min_score=DEFAULT_MIN_SCORE,
    max_position_distance=DEFAULT_MAX_POSITION_DISTANCE,
    axis_alignment_deg=DEFAULT_AXIS_ALIGNMENT_DEG,
):
    """Filter candidates by score, proximity, and object-edge alignment.

    GPD's ``position`` is at the base of the fingers, so a valid position may
    sit up to one finger depth away from an object surface. Object axes are
    obtained from a minimal oriented bounding box, allowing the food box to be
    rotated relative to the camera or world frame.

    The input list is updated in place and a dictionary of rejection counts is
    returned for logging.
    """
    points = np.asarray(food_points, dtype=np.float64).reshape(-1, 3)
    if points.shape[0] < 4:
        raise ValueError(
            'At least four food points are required to filter grasps.'
        )
    if max_position_distance < 0.0:
        raise ValueError('max_position_distance must be non-negative.')
    if not 0.0 <= axis_alignment_deg <= 90.0:
        raise ValueError('axis_alignment_deg must be between 0 and 90.')

    object_axes = _oriented_box_axes(points)
    alignment_cosine = math.cos(math.radians(axis_alignment_deg))
    kept = []
    rejected = {'score': 0, 'position': 0, 'alignment': 0}

    for grasp in grasp_config_list.grasps:
        if float(grasp.score.data) <= min_score:
            rejected['score'] += 1
            continue

        position = np.array(
            [grasp.position.x, grasp.position.y, grasp.position.z],
            dtype=np.float64,
        )
        nearest_distance = np.linalg.norm(points - position, axis=1).min()
        if nearest_distance > max_position_distance:
            rejected['position'] += 1
            continue

        grasp_axes = np.array(
            [
                [grasp.approach.x, grasp.binormal.x, grasp.axis.x],
                [grasp.approach.y, grasp.binormal.y, grasp.axis.y],
                [grasp.approach.z, grasp.binormal.z, grasp.axis.z],
            ],
            dtype=np.float64,
        )
        # Every gripper axis must be parallel to some edge of the food box.
        # Absolute dot products treat the two directions of an axis equally.
        best_alignment = np.max(np.abs(object_axes.T @ grasp_axes), axis=0)
        if np.any(best_alignment < alignment_cosine):
            rejected['alignment'] += 1
            continue

        kept.append(grasp)

    grasp_config_list.grasps = kept
    return rejected


def _oriented_box_axes(points):
    """Return the three unit edge directions of a minimal oriented box."""
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    box = cloud.get_minimal_oriented_bounding_box(robust=True)
    axes = np.array(box.R, dtype=np.float64, copy=True)
    axes /= np.linalg.norm(axes, axis=0, keepdims=True)
    return axes
