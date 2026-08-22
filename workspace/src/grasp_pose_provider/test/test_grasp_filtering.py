"""Tests for geometric and classifier filtering of GPD grasps."""

from types import SimpleNamespace

import numpy as np

from grasp_pose_provider.grasp_filtering import filter_grasps


def _vector(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


def _grasp(score=1.0, position=(0.0, 0.0, 0.0), angle_deg=0.0):
    angle = np.radians(angle_deg)
    cosine, sine = np.cos(angle), np.sin(angle)
    return SimpleNamespace(
        score=SimpleNamespace(data=score),
        position=_vector(*position),
        approach=_vector(cosine, sine, 0.0),
        binormal=_vector(-sine, cosine, 0.0),
        axis=_vector(0.0, 0.0, 1.0),
    )


def _box_points():
    corners = [
        [x, y, z]
        for x in (-0.035, 0.035)
        for y in (-0.0545, 0.0545)
        for z in (-0.0495, 0.0495)
    ]
    face_centers = [
        [0.035, 0.0, 0.0], [-0.035, 0.0, 0.0],
        [0.0, 0.0545, 0.0], [0.0, -0.0545, 0.0],
        [0.0, 0.0, 0.0495], [0.0, 0.0, -0.0495],
    ]
    return np.array(corners + face_centers)


def test_filters_score_distance_and_diagonal_orientation():
    valid = _grasp()
    low_score = _grasp(score=-0.1)
    far = _grasp(position=(0.2, 0.0, 0.0))
    diagonal = _grasp(angle_deg=35.0)
    grasp_list = SimpleNamespace(
        grasps=[valid, low_score, far, diagonal]
    )

    rejected = filter_grasps(
        grasp_list,
        _box_points(),
        max_position_distance=0.06,
        axis_alignment_deg=20.0,
    )

    assert grasp_list.grasps == [valid]
    assert rejected == {'score': 1, 'position': 1, 'alignment': 1}
