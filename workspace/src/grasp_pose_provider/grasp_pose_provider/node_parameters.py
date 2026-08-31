"""Configuration parameters for the grasp pose provider node.

:class:`GraspPoseProviderParameters` holds the default of every parameter it
declares, so the node's configuration reads from one place rather than from
the modules that consume it. ``config/grasp_pose_provider.yaml`` spells the
same values out for the launch file to pass in.

The node's parts take this object whole rather than individual values: see
``GraspCandidateGenerator``, ``GraspCandidateValidator`` and
``GraspCandidatePublisher``.
"""

from grasp_pose_provider.grasp_candidate_generation import stored_model


class GraspPoseProviderParameters:
    """Retrieve and store the grasp pose provider's ROS parameters."""

    # -- The scene the candidates are generated from --------------------------
    # The directory holding one stored empty-plate dump per camera, named after
    # the camera that recorded it. Resolved rather than spelled out: it is the
    # copy installed with the package, whose path depends on the install space.
    DEFAULT_STORED_POINTCLOUD_DIR = stored_model.default_stored_pointcloud_dir()
    # The point cloud topics of the three on-board cameras. The first one is
    # the reference: its frame is the one everything is merged into.
    DEFAULT_CAPTURED_TOPICS = (
        '/left_camera/points',
        '/left_back_camera/points',
        '/left_front_camera/points',
    )
    # How long to block waiting for a message on each camera topic (seconds).
    # Rendering all the simulated cameras currently takes roughly 25 seconds.
    DEFAULT_CAPTURE_WAIT_TIMEOUT_SEC = 120.0
    # The frame the returned grasp poses are expressed in.
    DEFAULT_BASE_FRAME = 'base_link'
    # How long a tf2 lookup blocks waiting for its transform (seconds).
    DEFAULT_TF_TIMEOUT_SEC = 5.0
    # How much history the tf2 buffer keeps (seconds). The camera frames are
    # static, so this only needs to cover a stale cloud stamp.
    DEFAULT_TF_CACHE_TIME_SEC = 30.0

    # -- Food segmentation ----------------------------------------------------
    # ICP max correspondence distance (metres) when registering the stored
    # empty-plate model onto the captured scene: pairs farther apart than this
    # are ignored.
    DEFAULT_ICP_MAX_CORRESPONDENCE_DISTANCE = 0.05
    # A captured point closer than this (metres) to the registered model counts
    # as explained by the plate and is dropped; what survives is candidate food.
    DEFAULT_FOOD_SUBTRACTION_DISTANCE_THRESHOLD = 0.01
    # DBSCAN settings for separating the disconnected regions the subtraction
    # leaves behind: the neighbourhood radius (metres) and the smallest number
    # of points that can form a cluster.
    DEFAULT_FOOD_CLUSTER_EPS = 0.02
    DEFAULT_FOOD_CLUSTER_MIN_POINTS = 10
    # Shelves and walls are approximately constant along at least one base_link
    # axis, while large subtraction artifacts can be much bigger than food. A
    # food cluster must fit between these dimensions (metres) along every
    # base_link axis.
    DEFAULT_MIN_FOOD_CLUSTER_AXIS_SPAN = 0.01
    DEFAULT_MAX_FOOD_CLUSTER_AXIS_SPAN = 0.15
    # The two percentiles a cluster's extent is measured between. The middle
    # 90% is used so a few depth-camera outliers do not decide the result.
    DEFAULT_FOOD_CLUSTER_SPAN_PERCENTILES = (5.0, 95.0)
    # Do not ask GPD to find a grasp when segmentation produced too little
    # evidence of food. This also filters isolated depth-camera noise.
    DEFAULT_MIN_FOOD_POINT_COUNT = 10
    # The cloud MoveIt builds its octomap from; keep in step with the topic in
    # moveit_config/config/sensors_3d.yaml.
    DEFAULT_FOOD_CLOUD_TOPIC = '/grasp_pose_provider/food_points'

    # -- The GPD grasp-detection service --------------------------------------
    DEFAULT_GPD_SERVICE_NAME = 'detect_constrained_grasps'
    # How long to wait for the GPD service to be available / to answer
    # (seconds). A full detection on the merged three-camera cloud has been
    # measured at ~130 s end to end, so this stays well clear of two minutes.
    DEFAULT_GPD_SERVICE_TIMEOUT_SEC = 600.0
    # Only local scene geometry is needed to evaluate grasps around the food.
    DEFAULT_GPD_CLOUD_CROP_RADIUS = 0.10
    # Must stay in step with the jaw plates in robot_description's gripper.xacro
    # and with hand_depth in gpd_ros2's ros_eigen_params.cfg: the plates are
    # 80 mm long and arm_tcp sits at their mid-height, 40 mm short of the tips.
    DEFAULT_GPD_HAND_DEPTH = 0.08
    DEFAULT_FINGER_TIP_FROM_TCP = 0.04
    DEFAULT_TCP_FROM_FINGER_BASE = (
        DEFAULT_GPD_HAND_DEPTH - DEFAULT_FINGER_TIP_FROM_TCP
    )

    # -- Publishing the candidates for RViz -----------------------------------
    DEFAULT_GRASP_POSES_TOPIC = '/grasp_pose_candidates'
    # How many of the ranked candidates are published for RViz.
    DEFAULT_MAX_RVIZ_CANDIDATES = 5

    # -- Validating the candidates against MoveIt -----------------------------
    DEFAULT_REACHABILITY_ACTION_NAME = '/check_pose_reachability'
    # How far down the ranked list to keep asking the arm for a plan.
    DEFAULT_MAX_REACHABILITY_CANDIDATES = 40
    # How long to wait for the arm's action server, and for one plan (seconds).
    DEFAULT_REACHABILITY_SERVER_TIMEOUT_SEC = 10.0
    DEFAULT_REACHABILITY_RESULT_TIMEOUT_SEC = 180.0

    # -- Pausing physics around the capture -----------------------------------
    # How long to wait for the physics-sync node's pause/resume services.
    DEFAULT_SIMULATION_SERVICE_TIMEOUT_SEC = 10.0

    def __init__(self, node):
        self._node = node

    def get_parameters(self):
        """Declare the node's parameters and store their resolved values."""
        self.stored_pointcloud_dir = self._declare(
            'stored_pointcloud_dir', self.DEFAULT_STORED_POINTCLOUD_DIR
        )
        self.captured_topics = list(
            self._declare(
                'captured_topics', list(self.DEFAULT_CAPTURED_TOPICS)
            )
        )
        self.capture_wait_timeout_sec = self._declare(
            'capture_wait_timeout_sec',
            self.DEFAULT_CAPTURE_WAIT_TIMEOUT_SEC,
        )
        self.base_frame = self._declare('base_frame', self.DEFAULT_BASE_FRAME)
        self.tf_timeout_sec = self._declare(
            'tf_timeout_sec', self.DEFAULT_TF_TIMEOUT_SEC
        )
        self.tf_cache_time_sec = self._declare(
            'tf_cache_time_sec', self.DEFAULT_TF_CACHE_TIME_SEC
        )

        self.icp_max_correspondence_distance = self._declare(
            'icp_max_correspondence_distance',
            self.DEFAULT_ICP_MAX_CORRESPONDENCE_DISTANCE,
        )
        self.food_subtraction_distance_threshold = self._declare(
            'food_subtraction_distance_threshold',
            self.DEFAULT_FOOD_SUBTRACTION_DISTANCE_THRESHOLD,
        )
        self.food_cluster_eps = self._declare(
            'food_cluster_eps', self.DEFAULT_FOOD_CLUSTER_EPS
        )
        self.food_cluster_min_points = self._declare(
            'food_cluster_min_points', self.DEFAULT_FOOD_CLUSTER_MIN_POINTS
        )
        self.min_food_cluster_axis_span = self._declare(
            'min_food_cluster_axis_span',
            self.DEFAULT_MIN_FOOD_CLUSTER_AXIS_SPAN,
        )
        self.max_food_cluster_axis_span = self._declare(
            'max_food_cluster_axis_span',
            self.DEFAULT_MAX_FOOD_CLUSTER_AXIS_SPAN,
        )
        self.food_cluster_span_percentiles = list(
            self._declare(
                'food_cluster_span_percentiles',
                list(self.DEFAULT_FOOD_CLUSTER_SPAN_PERCENTILES),
            )
        )
        self.min_food_point_count = self._declare(
            'min_food_point_count', self.DEFAULT_MIN_FOOD_POINT_COUNT
        )
        self.food_cloud_topic = self._declare(
            'food_cloud_topic', self.DEFAULT_FOOD_CLOUD_TOPIC
        )

        self.gpd_service_name = self._declare(
            'gpd_service_name', self.DEFAULT_GPD_SERVICE_NAME
        )
        self.gpd_service_timeout_sec = self._declare(
            'gpd_service_timeout_sec', self.DEFAULT_GPD_SERVICE_TIMEOUT_SEC
        )
        self.gpd_cloud_crop_radius = self._declare(
            'gpd_cloud_crop_radius', self.DEFAULT_GPD_CLOUD_CROP_RADIUS
        )
        self.tcp_from_finger_base = self._declare(
            'tcp_from_finger_base', self.DEFAULT_TCP_FROM_FINGER_BASE
        )

        self.grasp_poses_topic = self._declare(
            'grasp_poses_topic', self.DEFAULT_GRASP_POSES_TOPIC
        )
        self.max_rviz_candidates = self._declare(
            'max_rviz_candidates', self.DEFAULT_MAX_RVIZ_CANDIDATES
        )

        self.reachability_action_name = self._declare(
            'reachability_action_name',
            self.DEFAULT_REACHABILITY_ACTION_NAME,
        )
        self.max_reachability_candidates = self._declare(
            'max_reachability_candidates',
            self.DEFAULT_MAX_REACHABILITY_CANDIDATES,
        )
        self.reachability_server_timeout_sec = self._declare(
            'reachability_server_timeout_sec',
            self.DEFAULT_REACHABILITY_SERVER_TIMEOUT_SEC,
        )
        self.reachability_result_timeout_sec = self._declare(
            'reachability_result_timeout_sec',
            self.DEFAULT_REACHABILITY_RESULT_TIMEOUT_SEC,
        )

        self.simulation_service_timeout_sec = self._declare(
            'simulation_service_timeout_sec',
            self.DEFAULT_SIMULATION_SERVICE_TIMEOUT_SEC,
        )

    def _declare(self, name, default):
        """Declare one parameter and return its resolved value."""
        return self._node.declare_parameter(name, default).value
