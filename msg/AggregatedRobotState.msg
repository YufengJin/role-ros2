# Aggregated Robot State Message
# Contains combined state from multiple robots/grippers
# Published by robot_state_aggregator_node

# Header with timestamp
std_msgs/Header header

# Number of robots in the aggregated state
uint32 num_robots

# Robot namespaces
string[] robot_namespaces

# Combined joint positions (all robots)
float64[] joint_positions

# Combined joint velocities (all robots)
float64[] joint_velocities

# Combined joint names (all robots)
string[] joint_names

# Individual arm states (one per robot)
ArmState[] arm_states

# Individual gripper states (one per robot)
PolymetisGripperState[] gripper_states

