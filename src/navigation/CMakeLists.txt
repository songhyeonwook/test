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
  scripts/goal_error_recorder_HY.py
  scripts/dynamic_obstacle_tracker.py
  scripts/directional_collision_gate_HY.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
