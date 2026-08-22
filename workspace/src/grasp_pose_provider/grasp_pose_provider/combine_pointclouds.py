"""Capture the three camera clouds and patch them into a single cloud.

The robot carries three left-facing cameras -- the centre one plus a forward
and a rearward one flanking it -- each publishing its own
``sensor_msgs/msg/PointCloud2`` in its own optical frame. This module waits for
one message on each topic, asks
:class:`grasp_pose_provider.camera_transforms.CameraTransformResolver` where
each camera sits relative to the reference camera, moves every cloud into that
reference frame, and concatenates them.

The result is a single unorganized XYZ cloud in the reference frame -- by
default ``left_camera_frame``, the frame
:mod:`grasp_pose_provider.stored_model` merges the stored empty-plate dumps
into as well -- so everything downstream (ICP registration, subtraction, the
GPD request) keeps working on it unchanged.

NaN points are dropped while merging, so indices into the combined message are
dense and contiguous. Alongside the message, :func:`capture_combined_cloud`
reports which camera contributed each point and where each camera sits in the
reference frame, which is what the GPD server needs to orient surface normals
correctly for a multi-view cloud.
"""

import numpy as np
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2

from grasp_pose_provider import camera_transforms, pointcloud_conversion


# The point cloud topics of the three on-board cameras. The first one is the
# reference: its frame is the one everything is merged into.
DEFAULT_CAMERA_TOPICS = (
    '/left_camera/points',
    '/left_back_camera/points',
    '/left_front_camera/points',
)
# How long to block waiting for a message on each topic (seconds).
DEFAULT_WAIT_TIMEOUT_SEC = 10.0


class CombinedCloud:
    """One merged cloud plus the per-camera bookkeeping that produced it.

    ``msg`` is the merged ``sensor_msgs/msg/PointCloud2`` in ``frame_id``.
    ``camera_source`` is an ``int`` array with one entry per point of ``msg``,
    giving the index of the camera that saw it. ``view_points`` is an
    ``(n_cameras, 3)`` array with each camera's origin expressed in
    ``frame_id``; ``camera_source`` indexes into it.
    """

    def __init__(self, msg, camera_source, view_points, frame_id, topics):
        self.msg = msg
        self.camera_source = camera_source
        self.view_points = view_points
        self.frame_id = frame_id
        self.topics = topics


def capture_combined_cloud(
    node,
    transform_resolver,
    topics=DEFAULT_CAMERA_TOPICS,
    reference_frame=None,
    wait_timeout_sec=DEFAULT_WAIT_TIMEOUT_SEC,
    tf_timeout_sec=camera_transforms.DEFAULT_TF_TIMEOUT_SEC,
    feedback_cb=None,
):
    """Wait for one cloud on each topic and merge them into a single cloud.

    ``transform_resolver`` is a
    :class:`~grasp_pose_provider.camera_transforms.CameraTransformResolver`;
    it supplies the camera-to-camera transforms from the robot state
    publisher. ``reference_frame`` defaults to the frame of the first topic's
    cloud. ``feedback_cb``, when given, is called with a short progress string
    per camera.

    Returns a :class:`CombinedCloud`. Raises ``RuntimeError`` if any topic does
    not produce a message within ``wait_timeout_sec``, and lets
    ``tf2_ros.TransformException`` through if a camera transform never
    arrives.
    """
    messages = [
        _wait_for_cloud(node, topic, wait_timeout_sec, feedback_cb)
        for topic in topics
    ]

    if reference_frame is None:
        reference_frame = messages[0].header.frame_id

    point_blocks = []
    source_blocks = []
    view_points = []
    for index, (topic, message) in enumerate(zip(topics, messages)):
        _emit(
            feedback_cb,
            f"Transforming '{topic}' from '{message.header.frame_id}' into "
            f"'{reference_frame}'",
        )
        matrix = transform_resolver.lookup_matrix(
            reference_frame,
            message.header.frame_id,
            timeout_sec=tf_timeout_sec,
        )

        points = camera_transforms.apply_transform(
            matrix, pointcloud_conversion.finite_points(message)
        )
        point_blocks.append(points)
        source_blocks.append(np.full(points.shape[0], index, dtype=np.int64))
        # The camera sits at its own optical-frame origin, so its viewpoint in
        # the reference frame is just the translation part of the transform.
        view_points.append(matrix[:3, 3])

    combined_points = (
        np.concatenate(point_blocks) if point_blocks else np.empty((0, 3))
    )
    if combined_points.shape[0] == 0:
        raise RuntimeError(
            'The three camera clouds contained no finite points; nothing to '
            'merge.'
        )

    msg = pointcloud_conversion.numpy_to_ros(
        combined_points,
        frame_id=reference_frame,
        stamp=messages[0].header.stamp,
    )
    _emit(
        feedback_cb,
        f'Merged {len(topics)} camera clouds into {msg.width} points in '
        f"'{reference_frame}'",
    )
    return CombinedCloud(
        msg=msg,
        camera_source=np.concatenate(source_blocks),
        view_points=np.asarray(view_points, dtype=np.float64).reshape(-1, 3),
        frame_id=reference_frame,
        topics=tuple(topics),
    )


def _wait_for_cloud(node, topic, wait_timeout_sec, feedback_cb):
    """Block until one ``PointCloud2`` arrives on ``topic``, or time out."""
    _emit(feedback_cb, f"Waiting for a point cloud on '{topic}'")
    received, message = wait_for_message(
        PointCloud2,
        node,
        topic,
        time_to_wait=wait_timeout_sec,
    )
    if not received:
        raise RuntimeError(
            f'Timed out after {wait_timeout_sec}s waiting for a PointCloud2 on '
            f"'{topic}'."
        )
    return message


def _emit(feedback_cb, stage):
    """Forward a progress ``stage`` to ``feedback_cb`` if one was provided."""
    if feedback_cb is not None:
        feedback_cb(stage)
