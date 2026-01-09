# Robot State Message (V2)
# Contains combined state from robot arm(s) and gripper(s)
# Published by robot_state_aggregator_node

# Header with timestamp
std_msgs/Header header

# Number of robots/arms in the aggregated state
uint32 num_robots

# Robot namespaces
string[] robot_namespaces

# Combined joint positions (all robots)
float64[] joint_positions

# Combined joint velocities (all robots)
float64[] joint_velocities

# Combined joint names (all robots)
string[] joint_names

# Individual arm states (one per robot arm)
ArmState[] arm_states

# Individual gripper states (one per gripper)
GripperState[] gripper_states

