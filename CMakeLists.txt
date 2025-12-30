cmake_minimum_required(VERSION 3.8)
project(role_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find Python to get version for installation paths
find_package(Python3 COMPONENTS Interpreter REQUIRED)
if(Python3_VERSION_MAJOR AND Python3_VERSION_MINOR)
  set(PYTHON_VERSION "${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")
else()
  # Fallback: try to get version from interpreter
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import sys; print('{}.{}'.format(sys.version_info.major, sys.version_info.minor))"
    OUTPUT_VARIABLE PYTHON_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(NOT PYTHON_VERSION)
    # Last resort: default to 3.8 for ROS2 Foxy
    set(PYTHON_VERSION "3.8")
    message(WARNING "Could not detect Python version, defaulting to 3.8")
  endif()
endif()
message(STATUS "Detected Python version: ${PYTHON_VERSION}")

# Generate messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OculusButtons.msg"
  "msg/PolymetisRobotCommand.msg"
  "msg/PolymetisRobotState.msg"
  "msg/PolymetisGripperState.msg"
  "msg/PolymetisCommand.msg"
  "msg/GripperCommand.msg"
  "msg/ControllerStatus.msg"
  "srv/Reset.srv"
  "srv/Home.srv"
  "srv/StartCartesianImpedance.srv"
  "srv/StartJointImpedance.srv"
  "srv/TerminatePolicy.srv"
  "srv/MoveToJointPositions.srv"
  "srv/MoveToEEPose.srv"
  "srv/SolveIK.srv"
  "srv/ComputeFK.srv"
  "srv/ComputeTimeToGo.srv"
  "action/MoveToPose.action"
  DEPENDENCIES std_msgs geometry_msgs
)

# Note: launch and launch_ros are Python packages, declared in package.xml only
# zed_wrapper and zed_components are runtime dependencies, declared in package.xml

# Install Python nodes
install(PROGRAMS
  scripts/oculus_reader_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME oculus_reader_node
)
install(PROGRAMS
  scripts/teleoperation_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleoperation_node
)
install(PROGRAMS
  scripts/polymetis_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME polymetis_bridge
)
install(PROGRAMS
  scripts/fake_joint_states_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_joint_states_publisher
)
# Install test scripts
install(PROGRAMS
  tests/test_ros2_communication.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package
# Source directory is 'role_ros2' (underscore) matching Python import convention
# 
# IMPORTANT: Install to the same location as ROS2 messages/services to avoid import conflicts.
# ROS2 messages/services are automatically installed by rosidl_generate_interfaces, and their
# location varies by ROS2 version:
# - ROS2 Foxy: lib/python${PYTHON_VERSION}/site-packages
# - ROS2 Humble: local/lib/python${PYTHON_VERSION}/dist-packages
#
# We detect the ROS2 installation pattern by checking if local/dist-packages exists after
# rosidl_generate_interfaces runs. If it does, we install to the same location.
# Otherwise, we use the standard site-packages location.

# Check if ROS2 uses local/dist-packages (ROS2 Humble pattern)
# This is determined by checking the ament_cmake environment
# For ROS2 Humble, messages go to local/lib/python${PYTHON_VERSION}/dist-packages
# For ROS2 Foxy, messages go to lib/python${PYTHON_VERSION}/site-packages

# Try to detect ROS2 version pattern
# ROS2 Humble uses: local/lib/python${PYTHON_VERSION}/dist-packages
# ROS2 Foxy uses: lib/python${PYTHON_VERSION}/site-packages
# We'll install to both locations to ensure compatibility, but prioritize the one ROS2 uses

# Install to standard site-packages (ROS2 Foxy pattern)
install(DIRECTORY role_ros2
  DESTINATION lib/python${PYTHON_VERSION}/site-packages
  FILES_MATCHING 
    PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
)

# Also install to local/dist-packages (ROS2 Humble pattern)
# This ensures the package is found regardless of ROS2 version
install(DIRECTORY role_ros2
  DESTINATION local/lib/python${PYTHON_VERSION}/dist-packages
  FILES_MATCHING 
    PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files to robot/franka/fr3/
install(FILES
  role_ros2/robot_ik/franka/fr3.urdf
  DESTINATION share/${PROJECT_NAME}/robot/franka/fr3
)

# Install XML model files (needed by RobotIKSolver)
# Install to Python package directory so arm.py can find it at runtime
# Install to both site-packages (ROS2 Foxy) and dist-packages (ROS2 Humble)
install(FILES
  role_ros2/robot_ik/franka/fr3.xml
  DESTINATION lib/python${PYTHON_VERSION}/site-packages/role_ros2/robot_ik/franka
)

# Also install to dist-packages for ROS2 Humble compatibility
install(FILES
  role_ros2/robot_ik/franka/fr3.xml
  DESTINATION local/lib/python${PYTHON_VERSION}/dist-packages/role_ros2/robot_ik/franka
)

# Install mesh directory (needed by fr3.xml)
# Install to Python package directory to match fr3.xml location
install(DIRECTORY role_ros2/robot_ik/franka/mesh
  DESTINATION lib/python${PYTHON_VERSION}/site-packages/role_ros2/robot_ik/franka
  FILES_MATCHING
    PATTERN "*.obj"
)

# Also install mesh to dist-packages for ROS2 Humble compatibility
install(DIRECTORY role_ros2/robot_ik/franka/mesh
  DESTINATION local/lib/python${PYTHON_VERSION}/dist-packages/role_ros2/robot_ik/franka
  FILES_MATCHING
    PATTERN "*.obj"
)

# Install texture directory (needed by fr3.xml)
# Install to Python package directory to match fr3.xml location
install(DIRECTORY role_ros2/robot_ik/franka/texture
  DESTINATION lib/python${PYTHON_VERSION}/site-packages/role_ros2/robot_ik/franka
  FILES_MATCHING
    PATTERN "*.png"
)

# Also install texture to dist-packages for ROS2 Humble compatibility
install(DIRECTORY role_ros2/robot_ik/franka/texture
  DESTINATION local/lib/python${PYTHON_VERSION}/dist-packages/role_ros2/robot_ik/franka
  FILES_MATCHING
    PATTERN "*.png"
)

# Also install mesh files to share directory for compatibility
install(DIRECTORY role_ros2/robot_ik/franka/mesh
  DESTINATION share/${PROJECT_NAME}/robot/franka/fr3
  FILES_MATCHING
    PATTERN "*.obj"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

