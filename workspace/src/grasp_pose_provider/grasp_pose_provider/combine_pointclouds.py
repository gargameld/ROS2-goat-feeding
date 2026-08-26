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

import threading
import time

import numpy as np
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
# Rendering all four simulated cameras currently takes roughly 25 seconds on
# the target machine.  Leave enough margin for a complete publish cycle when a
# caller explicitly asks for messages newer than a particular instant.
DEFAULT_WAIT_TIMEOUT_SEC = 120.0


class PointCloudSnapshotter:
    """Continuously cache the newest cloud from every camera topic.

    The old implementation created a temporary subscription for one topic at
    a time.  Slow camera publishing meant that a subscriber could be created
    just after a frame and destroyed just before the next one, making action
    success depend on retry timing.  Persistent simultaneous subscriptions
    avoid that race and also let callers request a genuinely fresh set after
    a long-running GPD call.
    """

    def __init__(
        self,
        node,
        topics=DEFAULT_CAMERA_TOPICS,
        callback_group=None,
    ):
        """Subscribe to ``topics`` and retain their newest messages."""
        self._node = node
        self.topics = tuple(topics)
        self._condition = threading.Condition()
        self._messages = {}
        self._sequences = {topic: 0 for topic in self.topics}
        # A best-effort reader can receive from both reliable simulation
        # publishers and best-effort physical depth cameras.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscriptions = [
            node.create_subscription(
                PointCloud2,
                topic,
                lambda message, topic=topic: self._receive(topic, message),
                qos,
                callback_group=callback_group,
            )
            for topic in self.topics
        ]

    def _receive(self, topic, message):
        with self._condition:
            self._messages[topic] = message
            self._sequences[topic] += 1
            self._condition.notify_all()

    def mark(self):
        """Return per-topic sequence numbers for a later fresh snapshot."""
        with self._condition:
            return dict(self._sequences)

    def wait_for_snapshot(self, timeout_sec, after_sequences=None):
        """Return cached messages, optionally requiring them after a mark."""
        if timeout_sec <= 0.0:
            raise ValueError('timeout_sec must be positive.')
        minimum = after_sequences or {}
        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while not self._snapshot_ready(minimum):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    missing = [
                        topic for topic in self.topics
                        if not self._has_message_after(topic, minimum)
                    ]
                    freshness = (
                        ' fresh' if after_sequences is not None else ''
                    )
                    raise RuntimeError(
                        f'Timed out after {timeout_sec}s waiting '
                        f'for{freshness} '
                        f'PointCloud2 messages on: {", ".join(missing)}.'
                    )
                self._condition.wait(timeout=remaining)
            return [self._messages[topic] for topic in self.topics]

    def _snapshot_ready(self, minimum):
        return all(
            self._has_message_after(topic, minimum)
            for topic in self.topics
        )

    def _has_message_after(self, topic, minimum):
        received = topic in self._messages
        sequence = self._sequences[topic]
        return received and sequence > minimum.get(topic, -1)

    def destroy(self):
        """Destroy the subscriptions owned by this snapshotter."""
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        self._subscriptions = []


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
    snapshotter=None,
    after_sequences=None,
    snapshot_captured_cb=None,
):
    """Wait for one cloud on each topic and merge them into a single cloud.

    ``snapshotter`` should normally be a node-lifetime
    :class:`PointCloudSnapshotter`.  ``after_sequences`` can be a value from
    :meth:`PointCloudSnapshotter.mark` to require all selected clouds to be
    newer than that mark. ``transform_resolver`` is a
    :class:`~grasp_pose_provider.camera_transforms.CameraTransformResolver`;
    it supplies the camera-to-camera transforms from the robot state
    publisher. ``reference_frame`` defaults to the frame of the first topic's
    cloud. ``feedback_cb``, when given, is called with a short progress string
    per camera.

    Returns a :class:`CombinedCloud`. Raises ``RuntimeError`` if any topic does
    not produce a message within ``wait_timeout_sec``, and lets
    ``tf2_ros.TransformException`` through if a camera transform never
    arrives. ``snapshot_captured_cb``, when provided, is called immediately
    after all requested messages arrive and before cloud processing begins.
    """
    topics = tuple(topics)
    if snapshotter is None:
        # Retain a self-contained API for utility callers.  All subscriptions
        # are nevertheless active at once, unlike the former sequential wait.
        snapshotter = PointCloudSnapshotter(node, topics)
        owns_snapshotter = True
    else:
        owns_snapshotter = False
        if topics != snapshotter.topics:
            raise ValueError(
                'capture topics must match the PointCloudSnapshotter topics.'
            )

    freshness = 'new ' if after_sequences is not None else ''
    _emit(
        feedback_cb,
        f'Waiting for {freshness}point clouds from all {len(topics)} cameras',
    )
    try:
        messages = snapshotter.wait_for_snapshot(
            wait_timeout_sec,
            after_sequences=after_sequences,
        )
        if snapshot_captured_cb is not None:
            snapshot_captured_cb()
    finally:
        if owns_snapshotter:
            snapshotter.destroy()

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


def _emit(feedback_cb, stage):
    """Forward a progress ``stage`` to ``feedback_cb`` if one was provided."""
    if feedback_cb is not None:
        feedback_cb(stage)
