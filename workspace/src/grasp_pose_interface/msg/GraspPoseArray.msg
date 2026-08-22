# Candidate grasp poses produced by one ProvideGraspPose action invocation.
# Each pose keeps its own frame. Consumers intentionally ignore the timestamps
# when displaying these as static candidates.
geometry_msgs/PoseStamped[] poses
