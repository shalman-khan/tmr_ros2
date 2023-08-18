cmake_minimum_required(VERSION 3.5)
project(tm_gazebo_cf)

find_package(ament_cmake REQUIRED)
find_package(tm_gazebo REQUIRED)


install(DIRECTORY
  config
  launch
  xacro
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
