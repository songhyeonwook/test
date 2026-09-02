cmake_minimum_required(VERSION 3.8)
project(lift_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/LiftMove.action"
  DEPENDENCIES action_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# rosidl_generate_interfaces 가 ${PROJECT_NAME} 이름의 타깃을 만들므로 실행파일 타깃은
# 다른 이름을 쓰고, 설치되는 이름만 lift_node 로 되돌린다 (ros2 run lift_node lift_node).
add_executable(lift_node_exe src/lift_node.cpp)
set_target_properties(lift_node_exe PROPERTIES OUTPUT_NAME lift_node)

ament_target_dependencies(lift_node_exe
  rclcpp
  rclcpp_action
  diagnostic_msgs
  std_msgs
  std_srvs
)

# 같은 패키지에서 만든 액션을 그 패키지의 실행파일이 쓴다
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(lift_node_exe "${cpp_typesupport_target}")

install(TARGETS
  lift_node_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/lift_set.py
  script/lift_teleop_key.py
  script/lift_action.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
