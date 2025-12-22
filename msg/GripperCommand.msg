# Gripper Command Message
# Used for sending gripper control commands

# Header with timestamp
std_msgs/Header header

# Target width (meters) - 0.0 = closed, max_width = open
float64 width

# Movement speed (0.0-1.0)
float64 speed

# Gripper force (0.0-1.0)
float64 force

# Whether to block until command completes
bool blocking

