# Polymetis Robot State Message
# Contains all information available from Polymetis RobotInterface and GripperInterface

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
float64[] ee_position     # [x, y, z]
float64[] ee_quaternion   # [x, y, z, w]
float64[] ee_euler        # [roll, pitch, yaw]

# Gripper state
float64 gripper_width                 # meters
float64 gripper_position              # normalized (0=closed, 1=open)
bool gripper_is_grasped
bool gripper_is_moving
bool gripper_prev_command_successful
int32 gripper_error_code

# Controller and system information
float64 prev_controller_latency_ms
bool prev_command_successful
bool is_running_policy

# Robot timestamp from Polymetis (robot-side), in nanoseconds
uint64 polymetis_timestamp_ns


