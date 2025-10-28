cmake_minimum_required(VERSION 3.8)
project(smart_car)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(smartcar_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install all of the needed directories
install(DIRECTORY
  launch
  urdf
  worlds
  scripts
  config
  map
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()