cmake_minimum_required(VERSION 3.8)
project(navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
