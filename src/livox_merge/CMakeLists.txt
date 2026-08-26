cmake_minimum_required(VERSION 3.8)
# PCL_ROOT 같은 <Pkg>_ROOT 변수를 find_package 가 쓰도록 한다.
# 안 켜면 cmake 가 무시하면서 CMP0074 경고를 낸다.
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(livox_merge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  add_compile_options(${OpenMP_CXX_FLAGS})
  link_libraries(${OpenMP_CXX_LIBRARIES})
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(merge_lidar_node src/merge_lidar_node.cpp)
ament_target_dependencies(
  merge_lidar_node
  rclcpp
  sensor_msgs
  pcl_conversions
  livox_ros_driver2
)
target_link_libraries(merge_lidar_node ${PCL_LIBRARIES})

install(TARGETS merge_lidar_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
