# Motion Plan - a joint trajectory for the robot to follow. This represents the motion over one
# covered by a single raster pattern. The 'approach' or 'departure' fields may be empty if the
# robot goes straight from one process area to another without returning to a home position.

trajectory_msgs/JointTrajectory approach
trajectory_msgs/JointTrajectory process
trajectory_msgs/JointTrajectory departure
