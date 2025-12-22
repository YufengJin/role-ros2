# Controller Status Message
# Status of the robot controller and policy

# Header with timestamp
std_msgs/Header header

# Whether a policy is currently running
bool is_running_policy

# Current controller mode
string controller_mode  # "cartesian_impedance", "joint_impedance", "none"

# Controller gains (if applicable)
float64[] kp  # Position gains
float64[] kd  # Velocity gains

