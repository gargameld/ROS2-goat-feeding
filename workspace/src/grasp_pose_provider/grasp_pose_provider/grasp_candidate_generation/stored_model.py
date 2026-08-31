"""The stored empty-plate model, recorded once from each of the three cameras.

The model the food detection subtracts from the scene is captured the same way
the scene itself is: one ``sensor_msgs/msg/PointCloud2`` per camera, dumped to
YAML with ``ros2 topic echo``. This module reads those three dumps, moves each
one into the reference camera's frame with
:class:`grasp_pose_provider.grasp_candidate_generation.camera_transforms.CameraTransformResolver`
-- the same transforms
:mod:`grasp_pose_provider.grasp_candidate_generation.combine_pointclouds` uses
for the live clouds -- and concatenates them into a single Open3D cloud.

Recording the plate from all three viewpoints matters because the captured
scene is now a three-camera merge: a single-camera model would only explain the
plate surface one camera can see, and everything the other two contribute would
survive the subtraction and be reported as food.

The dumps live in ``stored_pointcloud_data/`` and are named after their camera,
i.e. after the first segment of the camera's point cloud topic:
``/left_camera/points`` -> ``left_camera.yaml``. Each file is read for its own
``header.frame_id``, so the mapping from file to camera frame comes out of the
dump itself rather than being hard-coded here.
"""

import os

import numpy as np
import open3d as o3d
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from grasp_pose_provider.grasp_candidate_generation import (
    camera_transforms,
    pointcloud_conversion,
)


# Name of the directory holding the stored per-camera dumps, both in the
# package sources and in the share directory setup.py installs them to.
STORED_POINTCLOUD_DIRNAME = 'stored_pointcloud_data'
# Extension of the stored dumps.
STORED_POINTCLOUD_SUFFIX = '.yaml'

# The copy in the sources, resolved from this file's real path:
# .../grasp_pose_provider/grasp_pose_provider/grasp_candidate_generation/
#     stored_model.py
#   -> .../grasp_pose_provider/stored_pointcloud_data/
_PACKAGE_MODULE_DIR = os.path.dirname(
    os.path.dirname(os.path.realpath(__file__))
)
_SOURCE_STORED_POINTCLOUD_DIR = os.path.join(
    os.path.dirname(_PACKAGE_MODULE_DIR),
    STORED_POINTCLOUD_DIRNAME,
)


def default_stored_pointcloud_dir():
    """Return the directory holding the stored per-camera dumps.

    ``setup.py`` installs them into the package's share directory, so that is
    where they are looked up first. Only a run against sources that have not
    been installed -- a source checkout, or a build whose share directory
    predates the dumps being installed -- falls back to the copy sitting next
    to this module, which is reachable whenever the module itself is a symlink
    back into the source tree.
    """
    try:
        share_dir = get_package_share_directory('grasp_pose_provider')
    except (PackageNotFoundError, LookupError):
        return _SOURCE_STORED_POINTCLOUD_DIR
    installed_dir = os.path.join(share_dir, STORED_POINTCLOUD_DIRNAME)
    if os.path.isdir(installed_dir):
        return installed_dir
    return _SOURCE_STORED_POINTCLOUD_DIR


def camera_name(topic):
    """Return the camera name a point cloud ``topic`` belongs to.

    ``/left_camera/points`` -> ``left_camera``.
    """
    return topic.strip('/').split('/')[0]


def stored_pointcloud_paths(directory, topics):
    """Return the stored dump path expected for each camera topic.

    The file name is the camera's name plus
    :data:`STORED_POINTCLOUD_SUFFIX`, so the three on-board cameras give
    ``left_camera.yaml``, ``left_back_camera.yaml`` and
    ``left_front_camera.yaml``.
    """
    return [
        os.path.join(
            directory, camera_name(topic) + STORED_POINTCLOUD_SUFFIX
        )
        for topic in topics
    ]


def load_stored_model(
    parameters,
    transform_resolver,
    reference_frame,
    feedback_cb=None,
):
    """Load the per-camera dumps and merge them into one Open3D cloud.

    ``parameters`` is the node's
    :class:`~grasp_pose_provider.node_parameters.GraspPoseProviderParameters`;
    the dump directory and the camera topics come from it. That directory holds
    one dump per camera topic, named as :func:`stored_pointcloud_paths`
    describes. Every cloud is transformed from the frame recorded in its own
    header into ``reference_frame`` before being concatenated, so the result is
    directly comparable to the merged captured cloud.

    Raises ``FileNotFoundError`` if any expected dump is missing, and
    ``RuntimeError`` if the dumps together contain no finite points.
    """
    directory = parameters.stored_pointcloud_dir
    paths = stored_pointcloud_paths(
        directory, topics=parameters.captured_topics
    )
    missing = [path for path in paths if not os.path.isfile(path)]
    if missing:
        raise FileNotFoundError(
            'Missing stored point cloud dump(s): '
            + ', '.join(missing)
            + '. One dump per camera is expected, named after the camera '
            'that recorded it.'
        )

    point_blocks = []
    for path in paths:
        msg = pointcloud_conversion.read_stored_pointcloud_msg(path)
        _emit(
            feedback_cb,
            f"Loading stored cloud '{os.path.basename(path)}' from "
            f"'{msg.header.frame_id}' into '{reference_frame}'",
        )
        matrix = transform_resolver.lookup_matrix(
            reference_frame,
            msg.header.frame_id,
        )
        point_blocks.append(
            camera_transforms.apply_transform(
                matrix, pointcloud_conversion.finite_points(msg)
            )
        )

    points = np.concatenate(point_blocks) if point_blocks else np.empty((0, 3))
    if points.shape[0] == 0:
        raise RuntimeError(
            f"The stored point cloud dumps in '{directory}' contained no "
            'finite points; nothing to register against.'
        )

    _emit(
        feedback_cb,
        f'Merged {len(paths)} stored camera clouds into {points.shape[0]} '
        f"points in '{reference_frame}'",
    )

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    return cloud


def _emit(feedback_cb, stage):
    """Forward a progress ``stage`` to ``feedback_cb`` if one was provided."""
    if feedback_cb is not None:
        feedback_cb(stage)
