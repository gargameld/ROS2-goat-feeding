"""Everything that turns a scene into ranked TCP grasp candidates.

``GraspCandidateGenerator``, in
:mod:`grasp_pose_provider.grasp_candidate_generation.grasp_candidate_generator`,
is this subpackage's entry point and the only part
:mod:`grasp_pose_provider.node_initializer` constructs directly; the rest are
the stages it drives, from capturing the camera clouds through segmenting the
food to converting the GPD service's grasp configurations into poses.

Validating those candidates and publishing them are separate concerns and live
outside this subpackage, in
:mod:`grasp_pose_provider.grasp_candidate_validator` and
:mod:`grasp_pose_provider.grasp_candidate_publisher`.
"""
