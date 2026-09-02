# rbio TransferRobot 스크립트로 hw 스택을 띄울 때 source 한다.
#
#   source ~/hw/tools/rbio_env.sh
#   ~/ros/TransferRobot/scripts/manage_navigation_stack.sh start
#
# rbio 스크립트는 기본값으로 /home/rb/ros2_ws/src/bringup.launch.py 를 찾는다.
# 이 파일이 그 기대값을 hw 로 돌린다. rbio 코드는 수정하지 않는다.

HW_WS="${HW_WS:-$HOME/hw}"

# --- nav 워크스페이스 = hw ---
export TRANSFER_ROBOT_NAV_WORKSPACE="${HW_WS}"
export TRANSFER_ROBOT_NAV_LAUNCH="${HW_WS}/src/bringup/launch/bringup.launch.py"
# bringup.launch.py 의 app 인자 기본값. rbio 스크립트는 launch 인자를 못 넘기므로 env 로 켠다.
#   - Nav2 는 /cmd_vel_nav 까지만 내고 앱이 /cmd_vel 로 중계
#   - motor_node 는 센터링 뒤 자동 애커만(AUTONOMOUS), 도킹 속도 0.04/0.05
export HW_APP_MODE=true

# --- ROS 도메인 (rbio 앱/스크립트와 같은 30 번) ---
export TRANSFER_ROBOT_ROS_DOMAIN_ID=30
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# --- 라이다는 hw 가 띄운다 (앱 자동 실행 끔) ---
export TRANSFER_ROBOT_EXTERNAL_LIVOX_DRIVER=1
export TRANSFER_ROBOT_LIVOX_SDK_LIBRARY_DIR=/usr/local/lib   # hw 는 SDK 를 /usr/local 에 설치
# 앱이 드라이버를 직접 띄우게 되더라도(EXTERNAL=0) hw 와 같은 JSON(포트별 host_ip) 을 쓰게
export TRANSFER_ROBOT_LIVOX_CONFIG_PATH="${HW_WS}/src/livox_ros_driver2/config/multi_MID360_config.json"

# --- 나중에 rbio 앱(UI)까지 붙일 때 앱이 hw 토픽/외부파라미터를 쓰도록 ---
# 앱은 /livox/lidar* 토픽을 기본으로 Livox CustomMsg 로 구독한다. hw 드라이버는 xfer_format=0
# (PointCloud2) 로 내므로 포맷을 명시한다 - 빠뜨리면 타입이 안 맞아 앱이 라이다를 못 본다.
#   LiDAR 1 = 192.168.1.135 livox_top (앱 기본 토픽), 2 = 192.168.2.102 livox_front, 3 = 192.168.3.144 livox_rear
export TRANSFER_ROBOT_LIDAR_1_FORMAT=pointcloud2
export TRANSFER_ROBOT_LIDAR_2_FORMAT=pointcloud2
export TRANSFER_ROBOT_LIDAR_3_FORMAT=pointcloud2
export TRANSFER_ROBOT_LIDAR_2_TOPIC=/livox/lidar_192_168_2_102   # rbio 는 /livox/lidar 로 remap 했었다
export TRANSFER_ROBOT_LIDAR_2_X_M=0.697
export TRANSFER_ROBOT_LIDAR_2_Y_M=0.350
export TRANSFER_ROBOT_LIDAR_2_Z_M=0.320
export TRANSFER_ROBOT_LIDAR_2_YAW_DEG=90.0
export TRANSFER_ROBOT_LIDAR_3_X_M=-0.600
export TRANSFER_ROBOT_LIDAR_3_Y_M=-0.340
export TRANSFER_ROBOT_LIDAR_3_Z_M=0.320
export TRANSFER_ROBOT_LIDAR_3_YAW_DEG=-90.0

source /opt/ros/humble/setup.bash
[ -f "${HW_WS}/install/setup.bash" ] && source "${HW_WS}/install/setup.bash"
echo "rbio_env: NAV_WORKSPACE=${TRANSFER_ROBOT_NAV_WORKSPACE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
