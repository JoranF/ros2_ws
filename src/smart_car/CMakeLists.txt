cmake_minimum_required(VERSION 3.8)
project(smart_car)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install all of the needed directories
install(DIRECTORY
  launch
  urdf
  worlds
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()