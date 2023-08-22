cmake_minimum_required(VERSION 3.5)
project(tm_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz models worlds xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
