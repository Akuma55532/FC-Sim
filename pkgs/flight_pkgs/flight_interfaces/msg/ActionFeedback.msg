# Phase is descriptive; consumers must use Action terminal state for completion.
string phase
geometry_msgs/PoseStamped current_pose
geometry_msgs/PoseStamped target_pose
bool pose_valid
float64 position_error_m
float64 yaw_error_rad
