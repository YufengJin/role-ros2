# Polymetis Command Message
# Complete control command for both Arm and Gripper via Polymetis framework

# Header with timestamp
std_msgs/Header header

# ========== ARM CONTROL (7 DOF) ==========
# Joint positions (radians) - desired joint angles
float64[] joint_positions

# Joint velocities (rad/s) - desired joint velocities  
float64[] joint_velocities

# Joint torques (Nm) - desired joint torques for torque control
float64[] joint_torques

# Impedance gains - Position gains (stiffness) for each joint
float64[] kp

# Impedance gains - Velocity gains (damping) for each joint
float64[] kd

# Control mode flags for arm
bool use_position   # Use position control
bool use_velocity   # Use velocity feedforward
bool use_torque     # Use torque control
bool blocking       # Block until command completes (for position control)

# ========== GRIPPER CONTROL ==========
bool operate_gripper  # Whether to operate gripper in this command

# Gripper target width (meters) - 0.0 = closed, max_width = open
float64 gripper_width

# Gripper movement speed (0.0-1.0)
float64 gripper_speed

# Gripper force (0.0-1.0)
float64 gripper_force

# Whether to block until gripper command completes
bool gripper_blocking

