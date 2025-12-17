cmake_minimum_required(VERSION 3.8)
project(role_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OculusButtons.msg"
  DEPENDENCIES std_msgs
)

# Note: launch and launch_ros are Python packages, declared in package.xml only
# zed_wrapper and zed_components are runtime dependencies, declared in package.xml

# Install Python nodes
install(PROGRAMS
  scripts/oculus_reader_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME oculus_reader_node
)

# Install test scripts
install(PROGRAMS
  scripts/test_franka_controllers.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_franka_controllers
)
install(PROGRAMS
  scripts/test_cartesian_pose_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_cartesian_pose_control
)
install(PROGRAMS
  scripts/test_joint_position_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_joint_position_control
)
install(PROGRAMS
  scripts/test_joint_velocity_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_joint_velocity_control
)
install(PROGRAMS
  scripts/test_cartesian_velocity_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_cartesian_velocity_control
)
install(PROGRAMS
  scripts/test_required_components.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_required_components
)

# Install Python package
# Source directory is 'role_ros2' (underscore) matching Python import convention
# Install to both site-packages and local/dist-packages for ROS2 compatibility
install(DIRECTORY role_ros2
  DESTINATION lib/python3.10/site-packages
  FILES_MATCHING PATTERN "*.py"
)
# Also install to local/dist-packages for ROS2 message compatibility
install(DIRECTORY role_ros2
  DESTINATION local/lib/python3.10/dist-packages
  FILES_MATCHING PATTERN "*.py"
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

