# Cartesian Position Command Message
# Used for sending end-effector position commands to the arm controller

# Header with timestamp
std_msgs/Header header

# Target end-effector position [x, y, z] in meters
float64[] position

# Target end-effector orientation as quaternion [x, y, z, w]
float64[] orientation

# Optional time to go (seconds, 0 = use default)
float64 time_to_go

# Whether to block until command completes (for non-blocking servo mode, this is ignored)
bool blocking

