# Hand State Message
# Aggregated state for a multi-finger dexterous hand (e.g. Allegro Hand V4, 16 DoF).
# Published by allegro_hand_interface_node by relaying the official ros2_control
# /joint_states topic and the most recent grasp action result.

# Header with timestamp
std_msgs/Header header

# Joint states (16 DoF for Allegro V4: 4 fingers x 4 joints)
# Order matches the hand_joint_names parameter passed at launch
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts
float64[] temperatures

# Most recent posture/grasp action result (sticky until the next action completes)
bool stalled       # True iff the hand reported max effort with no further motion
bool reached_goal  # True iff the most recent action goal was reached
string last_primitive  # Last grasp primitive issued (e.g. "home", "envelop", or "" if none yet)

# Timestamp from interface (ROS time), in nanoseconds
uint64 timestamp_ns
