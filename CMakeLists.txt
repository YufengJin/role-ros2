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
  # ============================================================================
  # Legacy messages (V1 - for polymetis_bridge_node.py, franka_robot.launch.py)
  # ============================================================================
  "msg/OculusButtons.msg"
  "msg/PolymetisRobotCommand.msg"
  "msg/PolymetisRobotState.msg"
  "msg/PolymetisGripperState.msg"
  "msg/GripperCommand.msg"
  "msg/ControllerStatus.msg"
  "msg/VRPolicyAction.msg"
  "msg/AggregatedRobotState.msg"
  # ============================================================================
  # V2 messages (for franka_robot_interface_node.py, franka_gripper_interface_node.py)
  # ============================================================================
  # Controller command messages
  "msg/JointPositionCommand.msg"
  "msg/JointVelocityCommand.msg"
  "msg/CartesianPositionCommand.msg"
  "msg/CartesianVelocityCommand.msg"
  # State messages
  "msg/ArmState.msg"
  "msg/GripperState.msg"
  "msg/RobotState.msg"
  # ============================================================================
  # Legacy services (V1)
  # ============================================================================
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
  # ============================================================================
  # V2 services
  # ============================================================================
  "srv/StartJointVelocity.srv"
  "srv/GripperGoto.srv"
  "srv/GripperGrasp.srv"
  # ============================================================================
  # Actions
  # ============================================================================
  "action/MoveToPose.action"
  DEPENDENCIES std_msgs geometry_msgs
)

# Note: launch and launch_ros are Python packages, declared in package.xml only
# zed_wrapper and zed_components are runtime dependencies, declared in package.xml

# Install Python nodes (from nodes/ directory)
install(PROGRAMS
  nodes/oculus_reader_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME oculus_reader_node
)
# New refactored nodes
install(PROGRAMS
  nodes/franka_robot_interface_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_robot_interface
)
install(PROGRAMS
  nodes/franka_gripper_interface_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_gripper_interface
)
install(PROGRAMS
  nodes/robot_state_aggregator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_state_aggregator
)
install(PROGRAMS
  nodes/camera_calibration_tf_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_calibration_tf_publisher
)
install(PROGRAMS
  nodes/franka_gripper_interface_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_gripper_interface
)
install(PROGRAMS
  nodes/robot_state_aggregator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_state_aggregator
)

# Note: scripts/ directory contains pure Python scripts (not ROS2 nodes)
# They should be run directly with python3, e.g.:
#   python3 scripts/collect_trajectory_franka.py --task pick_and_place
#   python3 scripts/replay_trajectory.py --filepath /path/to/trajectory.h5

# Install Python package
# Install to both site-packages (ROS2 Foxy) and dist-packages (ROS2 Humble) for compatibility
install(DIRECTORY role_ros2
  DESTINATION lib/python${PYTHON_VERSION}/site-packages
  FILES_MATCHING 
    PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
)
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

# Install hardware scripts (kill_all, launch_robot, launch_gripper) for launch files
install(PROGRAMS
  hardware/franka/launch/kill_all.sh
  hardware/franka/launch/launch_robot.sh
  hardware/franka/launch/launch_gripper.sh
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files (for launch files to load)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING
    PATTERN "*.yaml"
    PATTERN "*.yml"
    PATTERN "*.rviz"
)

# ============================================================================
# Robot Model Files (URDF, Mesh, XML)
# ============================================================================

# Install URDF files to robot_ik/franka/ (for launch file to load)
install(FILES
  role_ros2/robot_ik/franka/fr3.urdf
  role_ros2/robot_ik/franka/fr3_w_fin.urdf
  role_ros2/robot_ik/franka/fr3_w_soft_fin.urdf
  role_ros2/robot_ik/franka/bimanual_fr3_w_soft_fin.urdf
  DESTINATION share/${PROJECT_NAME}/robot_ik/franka
)

# Install mesh files to share directory (for URDF visualization in rviz)
# URDF files use: package://role_ros2/robot_ik/franka/mesh/link0.obj
# This installs mesh files to: share/role_ros2/robot_ik/franka/mesh/
# Note: Using trailing slash in source directory to install contents directly
install(DIRECTORY role_ros2/robot_ik/franka/mesh/
  DESTINATION share/${PROJECT_NAME}/robot_ik/franka/mesh
  FILES_MATCHING
    PATTERN "*.obj"
)

# Install XML model files (needed by RobotIKSolver)
install(FILES
  role_ros2/robot_ik/franka/fr3.xml
  DESTINATION lib/python${PYTHON_VERSION}/site-packages/role_ros2/robot_ik/franka
)
install(FILES
  role_ros2/robot_ik/franka/fr3.xml
  DESTINATION local/lib/python${PYTHON_VERSION}/dist-packages/role_ros2/robot_ik/franka
)

# Install mesh directory (needed by fr3.xml for RobotIKSolver)
# Note: Using trailing slash to install contents directly
install(DIRECTORY role_ros2/robot_ik/franka/mesh/
  DESTINATION lib/python${PYTHON_VERSION}/site-packages/role_ros2/robot_ik/franka/mesh
  FILES_MATCHING
    PATTERN "*.obj"
)
install(DIRECTORY role_ros2/robot_ik/franka/mesh/
  DESTINATION local/lib/python${PYTHON_VERSION}/dist-packages/role_ros2/robot_ik/franka/mesh
  FILES_MATCHING
    PATTERN "*.obj"
)

# Install texture directory (needed by fr3.xml for RobotIKSolver)
# Note: Using trailing slash to install contents directly
install(DIRECTORY role_ros2/robot_ik/franka/texture/
  DESTINATION lib/python${PYTHON_VERSION}/site-packages/role_ros2/robot_ik/franka/texture
  FILES_MATCHING
    PATTERN "*.png"
)
install(DIRECTORY role_ros2/robot_ik/franka/texture/
  DESTINATION local/lib/python${PYTHON_VERSION}/dist-packages/role_ros2/robot_ik/franka/texture
  FILES_MATCHING
    PATTERN "*.png"
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

