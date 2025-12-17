# Polymetis Robot Command Message
# Used for sending control commands to polymetis_manager

# Header with timestamp
std_msgs/Header header

# Control mode
string action_space  # "cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"
string gripper_action_space  # "position" or "velocity" (optional, defaults based on action_space)

# Command values
float64[] command  # Command array (7 for arm, 1 for gripper = 8 total, or 6 for cartesian + 1 gripper = 7)

# Control flags
bool blocking  # Whether to wait for command completion
bool velocity  # Whether command is velocity (for backward compatibility)

# Optional noise for randomization
float64[] cartesian_noise  # Optional cartesian noise for joint commands


