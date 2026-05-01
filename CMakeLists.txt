# Minimum CMake version required to build the project.
cmake_minimum_required(VERSION 3.8)

# Define the project name.
project(robot_patrol)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set additional compiler flags for GNU or Clang compilers to enforce stricter warnings.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies.
find_package(ament_cmake REQUIRED)       # Core package for building and installing ROS 2 packages.
find_package(rclcpp REQUIRED)            # ROS 2 C++ client library.
find_package(sensor_msgs REQUIRED)       # Messages for sensor data, such as LaserScan.
find_package(geometry_msgs REQUIRED)     # Messages for geometry data, such as Twist.
find_package(rosidl_default_generators REQUIRED)  # Generates code for ROS 2 interfaces.
find_package(rclcpp_action REQUIRED)     # Action client and server library in ROS 2.
find_package(action_msgs REQUIRED)       # Messages for action interfaces.
find_package(nav_msgs REQUIRED)          # Navigation-related messages, like Pose.

# Generate the necessary ROS interfaces (service and action) for this project.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"            # Service definition to get the direction.
  "action/GoToPose.action"          # Action definition to go to a specific pose.
  DEPENDENCIES sensor_msgs geometry_msgs  # Dependencies for the generated interfaces.
)

# Get the typesupport target for the generated interfaces, enabling ROS 2 to use the interfaces in C++.
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Add executables for each source file in the project.
add_executable(patrol_node src/patrol.cpp)                       # Patrol node using topics.
add_executable(direction_service src/direction_service.cpp)      # Direction service node.
add_executable(test_service src/test_service.cpp)                # Test service node.
add_executable(patrol_with_service src/patrol_with_service.cpp)  # Patrol node using the service.
add_executable(go_to_pose_action src/go_to_pose_action.cpp)      # Go-to-pose action node.

# Specify dependencies for each executable target.
ament_target_dependencies(patrol_node rclcpp sensor_msgs geometry_msgs) # Link rclcpp, sensor_msgs and geometry_msgs with patrol_node
target_link_libraries(patrol_node "${cpp_typesupport_target}")  # Link generated typesupport.

ament_target_dependencies(direction_service rclcpp)  # Link rclcpp with direction_service.
target_link_libraries(direction_service "${cpp_typesupport_target}")  # Link generated typesupport.

ament_target_dependencies(test_service rclcpp sensor_msgs)  # Link rclcpp and sensor_msgs with test_service.
target_link_libraries(test_service "${cpp_typesupport_target}")  # Link generated typesupport.

ament_target_dependencies(patrol_with_service rclcpp sensor_msgs)  # Link rclcpp and sensor_msgs with patrol_with_service.
target_link_libraries(patrol_with_service "${cpp_typesupport_target}")  # Link generated typesupport.

ament_target_dependencies(go_to_pose_action rclcpp rclcpp_action geometry_msgs nav_msgs)  # Link rclcpp, rclcpp_action, geometry_msgs, and nav_msgs with go_to_pose_action.
target_link_libraries(go_to_pose_action "${cpp_typesupport_target}")  # Link generated typesupport.

# Install the executables into the install directory under lib/${PROJECT_NAME}.
install(TARGETS patrol_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS direction_service DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_service DESTINATION lib/${PROJECT_NAME})
install(TARGETS patrol_with_service DESTINATION lib/${PROJECT_NAME})
install(TARGETS go_to_pose_action DESTINATION lib/${PROJECT_NAME})

# Install additional directories like launch and rviz configuration files into the share directory.
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies so that other packages can use this package's interfaces.
ament_export_dependencies(ament_cmake rclcpp sensor_msgs geometry_msgs)

# If testing is enabled, configure linting tools.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  # Automated linting tools for C++ and CMake.

  # The following lines disable copyright and cpplint checks. 
  # Uncomment these lines when copyrights are added to all source files.
  set(ament_cmake_copyright_FOUND TRUE)  # Skip copyright checks.
  set(ament_cmake_cpplint_FOUND TRUE)    # Skip cpplint checks.

  # Automatically find and add test dependencies.
  ament_lint_auto_find_test_dependencies()
endif()

# Macro that marks the package as an ament package, enabling ament-specific functionality.
ament_package()
