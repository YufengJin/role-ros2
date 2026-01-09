# Cartesian Velocity Command Message
# Used for sending end-effector velocity commands to the arm controller

# Header with timestamp
std_msgs/Header header

# Target end-effector linear velocity [vx, vy, vz] in m/s
float64[] linear_velocity

# Target end-effector angular velocity [wx, wy, wz] in rad/s
float64[] angular_velocity

