# Robot State Message (V2)
# Contains combined state from robot arm(s) and gripper(s)
# Published by robot_state_aggregator_node

# Header with timestamp
std_msgs/Header header

# Number of robots/arms in the aggregated state
uint32 num_robots

# Robot namespaces
string[] robot_namespaces

# Individual arm states (one per robot arm)
# Each ArmState contains joint_positions, joint_velocities, ee_pose, etc.
ArmState[] arm_states

# Individual gripper states (one per gripper)
GripperState[] gripper_states
