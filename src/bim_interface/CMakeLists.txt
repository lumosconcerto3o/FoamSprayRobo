cmake_minimum_required(VERSION 3.5)
project(bim_interface)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install scripts
install(PROGRAMS
  scripts/bim_interface_stub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
