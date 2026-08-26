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

# --- 나중에 rbio 앱(UI)까지 붙일 때 앱이 hw 토픽/외부파라미터를 쓰도록 ---
export TRANSFER_ROBOT_LIDAR_2_TOPIC=/livox/lidar_192_168_2_102
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
