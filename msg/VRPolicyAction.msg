# VR Policy Action Message
# Action computed from VR controller input for robot control

# Header with timestamp
std_msgs/Header header

# Action vector: [vx, vy, vz, wx, wy, wz, gripper_vel]
# - Linear velocities (m/s): [vx, vy, vz]
# - Angular velocities (rad/s): [wx, wy, wz]
# - Gripper velocity (normalized -1.0 to 1.0): [gripper_vel]
float64[7] action

# Controller state information
bool movement_enabled    # Whether movement is enabled (grip pressed)
bool controller_on       # Whether controller is active and receiving data
bool success             # Success button pressed (A for right, X for left)
bool failure             # Failure button pressed (B for right, Y for left)

# Joystick orientation reset state
bool joystick_pressed        # Whether joystick is currently pressed
bool orientation_resetting   # Whether orientation reset is in progress
float32 joystick_hold_progress  # Progress of joystick hold (0.0-1.0) for orientation reset
                                # 1.0 = long press completed, <1.0 = still holding

# Target positions (optional, for debugging/monitoring)
float64[6] target_cartesian_position  # [x, y, z, roll, pitch, yaw]
float64 target_gripper_position       # Normalized gripper position (0.0-1.0)

