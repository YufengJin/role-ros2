# Polymetis Gripper State Message
# Contains all information available from Polymetis GripperInterface

# Header with timestamp
std_msgs/Header header

# Gripper state
float64 width                 # meters
float64 position              # normalized (0=closed, 1=open)
bool is_grasped
bool is_moving
bool prev_command_successful
int32 error_code

# Metadata
float64 max_width             # meters

# Gripper timestamp from Polymetis (robot-side), in nanoseconds
uint64 polymetis_timestamp_ns


