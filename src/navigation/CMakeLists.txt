cmake_minimum_required(VERSION 3.8)
project(navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  behavior_trees
  map
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/top_tf_switcher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
