
-Motor-

source ~/ros2_ws/install/setup.bash

source ~/ws_livox/install/setup.bash

alias can='~/ros2_ws/src/motor_node/script/can.sh'

alias bringup='ros2 launch motor_node bringup.launch.py'

alias init='ros2 run motor_node drive_set.py '

alias monitor='ros2 run motor_node state_monitor.py'
-------------------------------------------------------------------------------------------------

-SLAM & Localization-

ros2 launch livox_ros_driver2 rviz_multi_launch.py

ros2 launch pointcloud_concatenate pointcloud_concatenate.launch.py

ros2 run pointcloud2_to_custommsg pointcloud2_to_custommsg_node

alias imu='ros2 launch myahrs_ros2_driver myahrs_ros2_driver.launch.py'

ros2 launch fast_lio mapping.launch.py 

ros2 launch fast_lio_localization localization.launch.py

rosbag record -a
-------------------------------------------------------------------------------------------------

-Navigation-

ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/cloud_registered_body \
  -r scan:=/scan \
  -p target_frame:=body

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false

