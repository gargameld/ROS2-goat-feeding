"""Publish the detected food points as the octomap's only input cloud.

MoveIt's occupancy map monitor builds the planning-scene octomap from whatever
point clouds ``moveit_config/config/sensors_3d.yaml`` points it at. Feeding it
the raw camera clouds fills the octomap with the whole shelf and everything
else in view, which is both expensive to collision-check and mostly redundant:
the static geometry is already declared as collision objects in
``environment_boxes.yaml``. This module republishes just the food points that
:func:`grasp_pose_provider.grasp_candidate_generation.food_detector.detect_food`
already segmented out of the combined cloud, and ``sensors_3d.yaml`` subscribes
to that topic alone.

The cloud goes out in the frame the combined cloud was merged into -- the
reference camera's frame -- so the occupancy map monitor gets a usable sensor
origin for its ray casting, and it keeps the capture timestamp so the monitor's
tf2 message filter can resolve that frame against the octomap frame. That means
it has to be published while the capture stamp is still inside move_group's tf2
buffer, i.e. before the long GPD call rather than after it.
"""

import numpy as np
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from grasp_pose_provider.grasp_candidate_generation import pointcloud_conversion


def food_cloud_message(cloud_msg, food_indices, stamp=None):
    """Return a ``PointCloud2`` holding only the food points of ``cloud_msg``.

    ``food_indices`` indexes into ``cloud_msg``'s point ordering with NaN
    points included -- the indexing
    :func:`~grasp_pose_provider.grasp_candidate_generation.food_detector.detect_food`
    produces. Points that are non-finite are dropped, so the result is a dense
    XYZ cloud in ``cloud_msg``'s frame. ``stamp`` defaults to ``cloud_msg``'s
    own stamp.
    """
    points = point_cloud2.read_points_numpy(
        cloud_msg, field_names=('x', 'y', 'z'), skip_nans=False
    )
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)

    indices = np.asarray(food_indices, dtype=np.int64).reshape(-1)
    food_points = points[indices]
    food_points = food_points[np.isfinite(food_points).all(axis=1)]

    return pointcloud_conversion.numpy_to_ros(
        food_points,
        frame_id=cloud_msg.header.frame_id,
        stamp=cloud_msg.header.stamp if stamp is None else stamp,
    )


class FoodCloudPublisher:
    """Owns the publisher the occupancy map monitor subscribes to.

    The publisher is created once and kept for the node's lifetime: the monitor
    subscribes when move_group starts, long before the first goal arrives, so
    creating it per goal would risk losing the single cloud each cycle
    publishes to discovery latency.
    """

    def __init__(self, node, parameters):
        """Advertise the node's ``food_cloud_topic``.

        ``parameters`` is the node's
        :class:`~grasp_pose_provider.node_parameters.GraspPoseProviderParameters`,
        which is where that topic comes from.
        """
        self._node = node
        topic = parameters.food_cloud_topic
        self._topic = topic
        # Sensor-data subscribers are best-effort, which a reliable publisher
        # is compatible with. Depth 1: only the newest food cloud matters.
        self._publisher = node.create_publisher(
            PointCloud2,
            topic,
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

    @property
    def topic(self):
        """The topic the food cloud is published on."""
        return self._topic

    def publish(self, cloud_msg, food_indices, stamp=None):
        """Publish the food points of ``cloud_msg`` and return the message."""
        msg = food_cloud_message(cloud_msg, food_indices, stamp=stamp)
        self._publisher.publish(msg)
        self._node.get_logger().info(
            f'Published {msg.width} food points to the octomap on '
            f"'{self._topic}' in frame '{msg.header.frame_id}'"
        )
        return msg

    def destroy(self):
        """Release the publisher owned by this helper."""
        self._node.destroy_publisher(self._publisher)
