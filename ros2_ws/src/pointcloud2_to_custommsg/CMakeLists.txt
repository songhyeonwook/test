cmake_minimum_required(VERSION 3.8)
project(pointcloud2_to_custommsg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud2_to_custommsg_node
  src/pointcloud2_to_custommsg.cpp
)

ament_target_dependencies(pointcloud2_to_custommsg_node
  rclcpp
  sensor_msgs
  livox_ros_driver2
  pcl_conversions
)

target_link_libraries(pointcloud2_to_custommsg_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  pointcloud2_to_custommsg_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
