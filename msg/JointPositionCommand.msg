# Joint Position Command Message
# Used for sending joint position commands to the arm controller

# Header with timestamp
std_msgs/Header header

# Target joint positions (7 DOF for Franka arm)
float64[] positions

# Optional time to go (seconds, 0 = use default)
float64 time_to_go

# Whether to block until command completes (for non-blocking servo mode, this is ignored)
bool blocking

