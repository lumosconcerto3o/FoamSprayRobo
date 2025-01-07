cmake_minimum_required(VERSION 3.5)
project(visualization)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install scripts
install(PROGRAMS
  scripts/visualize_spray_path.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
