std_msgs/Header header
string operation
string phase
uint64 operation_id
uint16 outcome_code
string detail
bool connected
bool telemetry_valid
bool pose_valid
bool armed
bool control_active
bool holding
# UNKNOWN=0, ON_GROUND=1, IN_AIR=2
uint8 ground_state
string backend_mode
geometry_msgs/Pose current_pose
geometry_msgs/Pose target_pose

# Retained task progress; index is zero-based and completed_count spans cycles.
bool mission_active
uint32 waypoint_index
uint32 completed_count
uint32 cycle_count
float64 dwell_elapsed_sec
