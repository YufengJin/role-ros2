# Gripper State Message (V2)
# Contains gripper state information for franka_gripper_interface_node

# Header with timestamp
std_msgs/Header header

# Gripper state
float64 width                 # meters (0.0 = closed, max_width = open)
float64 position              # normalized (0.0 = closed, 1.0 = open)
bool is_grasped               # True if object detected between fingers
bool is_moving                # True if gripper is moving
bool prev_command_successful  # True if previous command succeeded
int32 error_code              # Error code (0 = no error)

# Metadata
float64 max_width             # Maximum gripper width in meters

# Timestamp from interface (ROS time), in nanoseconds
uint64 timestamp_ns

