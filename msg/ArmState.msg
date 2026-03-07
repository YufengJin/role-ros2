# Arm State Message
# Contains comprehensive state information for the robot arm (without gripper)
# Used by franka_robot_interface_node

# Header with timestamp
std_msgs/Header header

# Joint states (7 DOF arm)
float64[] joint_positions
float64[] joint_velocities
float64[] joint_torques_computed
float64[] prev_joint_torques_computed
float64[] prev_joint_torques_computed_safened
float64[] motor_torques_measured

# End-effector pose (from forward kinematics)
# After aggregator transform these are in the target frame (e.g. base_link)
float64[] ee_position     # [x, y, z]
float64[] ee_quaternion   # [x, y, z, w]
float64[] ee_euler        # [roll, pitch, yaw]

# End-effector pose in the arm's own base frame (e.g. left_fr3_panda_link0)
# Preserved by aggregator without transform, required for IK computation
float64[] ee_position_local     # [x, y, z] in arm base frame
float64[] ee_quaternion_local   # [x, y, z, w] in arm base frame
float64[] ee_euler_local        # [roll, pitch, yaw] in arm base frame

# Controller and system information
float64 prev_controller_latency_ms
bool prev_command_successful
bool is_running_policy

# Controller mode
string controller_mode  # "cartesian_impedance", "joint_impedance", "joint_velocity", "none"

# Controller gains (if applicable)
float64[] kp  # Position gains
float64[] kd  # Velocity gains

# Robot timestamp from Polymetis (robot-side), in nanoseconds
uint64 polymetis_timestamp_ns

