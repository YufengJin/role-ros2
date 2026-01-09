# Joint Velocity Command Message
# Used for sending joint velocity commands to the arm controller

# Header with timestamp
std_msgs/Header header

# Target joint velocities (7 DOF for Franka arm, rad/s)
float64[] velocities

