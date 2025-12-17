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

# Install Python package
install(DIRECTORY role-ros2/role-ros2
  DESTINATION lib/python3.10/site-packages
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

