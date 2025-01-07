cmake_minimum_required(VERSION 3.5)
project(robot_insulation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(xacro REQUIRED)

# Add custom targets if needed
# Example: add_executable(${PROJECT_NAME}_node src/main.cpp)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch" EXCLUDE
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/generate_spray_path.py
  scripts/plan_point_to_point.py
  DESTINATION lib/${PROJECT_NAME}
)

# Add support for ament package
ament_package()
