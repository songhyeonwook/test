# hw — 4WS(후륜 역조향) 플랫폼

Livox MID-360 3대 + 내장 IMU로 **FAST-LIO 3D 측위**를 하고, 그 자세를
**평면으로 눌러서(z/roll/pitch 제거)** **Nav2 2D 내비게이션**에 넘기는 차량
한 대분의 전체 설정. 차량은 앞/뒤 축을 반대로 꺾는 **4WS(후륜 역조향)** 이고
Nav2 는 MPPI(Ackermann 모델) + Smac Hybrid-A* 로 돌린다.

```
hw/
├── src/                            colcon 워크스페이스 (여기 하나뿐)
│   ├── livox_ros_driver2/          MID-360 x3 드라이버 (PointCloud2, xfer_format=0)
│   ├── pointcloud_concatenate_ros2/ 라이다 3대를 TF 로 base_link 기준 병합
│   ├── fast_lio_localization/      FAST-LIO 측위 + ICP 전역정합 + tf_2d 평면화
│   ├── motor_node/                 CAN 4WS 구동. /mode 0 정렬·1 애커만·2 디프, /odom (4WS 휠 오도메트리)
│   ├── lift_node/                  RS232 리프트 제어보드. lift/vertical·lift/horizontal (Int32 0 정지/1/2),
│   │                                lift_node/move 액션 (시간 기반 구동·완료 판정)
│   ├── navigation/                 ★ URDF/TF · Nav2 파라미터 · behavior tree · 맵
│   └── bringup/                    ★ 전체 launch, bag 재생 테스트
├── third_party/Livox-SDK2/         colcon 대상 아님. multi-NIC 패치 적용됨, cmake --install 필요
├── tools/                          lidar_net.sh(라이다 포트/IP), can_setup.sh, check_frames.py, check_lidar_z.py, check_lidar_overlap.py, ply_to_pcd.py
└── bag/                            rosbag 기록 위치 (git 제외)
```

**Nav2 본체는 소스가 아니라 apt(`/opt/ros/humble`)에서 온다.** `src/navigation`
은 이 차량에 맞춰 실제로 손댄 것만 들고 있다.

```
src/navigation/
├── urdf/               차량/센서 형상. TF 의 유일한 출처 (mount.xacro 에 장착값)
├── rviz/               frames_check.rviz
├── launch/             description.launch.py (TF), navigation_launch.py (Nav2)
├── params/             nav2_params.yaml
├── behavior_trees/     차량형 전용 BT (spin/backup 없음, 5초 주기 재계획)
└── map/                test.pcd (3D, 측위용) + test.pgm/test.yaml (2D, 코스트맵용)
```

3D(`test.pcd`)와 2D(`test.pgm`)는 **같은 주행에서 나온 짝이어야** 한다.
어긋나면 로봇이 벽 속에 있다고 나온다.

---

## 데이터 흐름

```
MID-360 front ─┐  PointCloud2 (xfer_format=0, 점마다 절대시각 timestamp)
MID-360 rear  ─┤
MID-360 top  ──┴─ pointcloud_concat ─┬─ /livox_merge/merged_pointcloud         (전체)  ─┐
                  (TF 로 base_link)  └─ /livox_merge/merged_pointcloud_sliced  (z 슬라이스 + 차체 크롭) ─ Nav2 costmap
MID-360 top ───── /livox/imu_192_168_1_135 ──────────────────────────────────────────┤
                                                                                     │
                        fastlio_mapping        /Odometry (camera_init->body, 3D)  <──┘
                        global_localization    /map_to_odom (map->camera_init, ICP 0.5Hz)
                                    │                 │
                                    └── tf_2d.py ─────┘   토픽만 구독, 평면화
                                            │
                                  map ─ odom ─ base_link   (TF)
                                            │
                                          Nav2 ─ /cmd_vel ─ motor_node
```

TF 트리 (단일 체인):

```
map ── odom ── base_link ── livox_frame ─┬─ livox_top ── imu_link
               (tf_2d, 지면)  (라이다 중심)  ├─ livox_front
                                            └─ livox_rear
```

* `map->odom`, `odom->base_link` 는 `tf_2d.py` 가 낸다. `base_link` 는 z 를 0 으로
  누른 평면 프레임이자 **지면 프레임**이다. 차량 모델과 센서는 전부 `base_link`
  아래에 있고, `base_link` 기준 z 가 곧 지면 위 높이다.
  (예전에는 `base_link` 를 지면 0.5 m 위에 두고 `base_footprint` 를 따로 뒀는데,
  기준이 둘로 갈려 파라미터마다 -0.5 보정이 붙어서 2026-08-27 에 합쳤다.
  `/localization` 토픽의 `child_frame_id` 만 앱 호환을 위해 문자열로 남겨 뒀다.)
* 사전지도 PCD 의 바닥면은 `map` z ≈ -0.385 에 있는데 `planarize()` 가 z=0 을
  강제하므로, RViz 에서 3D prior map 을 겹쳐 보면 차량이 그만큼 떠 보인다.
  x/y/yaw 와 costmap 높이 필터에는 영향이 없다 (지도를 다시 만들 때 정리할 것).
* FAST-LIO 의 `camera_init` / `body` 는 **TF 에 올리지 않는다.** `body` 는 차량
  기준점이 아니라 상단 라이다 안에서 옆으로 누운 **IMU 프레임**이라, 그대로
  평면화하면 yaw 가 엉킨다. body -> base_link 변환은
  `fast_lio_localization/config/mid360.yaml` 의 `ref_from_body_*` 가 들고 있고
  `tf_2d.py` 와 `global_localization.py` 가 파라미터로 받는다.
  RViz 에서 camera_init 프레임 토픽(`/cloud_registered` 등)을 보고 싶을 때만
  `localization.launch.py debug_tf:=true`.
* 같은 기하가 두 곳에 있다: `mount.xacro`(원본)와 `mid360.yaml` 의
  `extrinsic_T/R`(base_link -> IMU) · `ref_from_body_*`(IMU -> base_link).
  병합 노드는 TF 를 그대로 읽으므로 장착값 사본을 들고 있지 않다.
  센서 위치를 바꾸면 `mount.xacro` 만 고친 뒤
  `python3 tools/check_frames.py` 로 나머지와의 정합을 확인한다.
  **top 을 옮기면 `imu_link` 도 같이 움직이므로 `mid360.yaml` 의 `extrinsic_T` 와
  `ref_from_body_xyz` 가 반드시 따라와야 한다** (check_frames.py 가 잡아준다).
  URDF 값 자체가 실물과 맞는지는 `tools/check_lidar_z.py`(z, 천장 평면 기준) 로
  본다. `check_lidar_overlap.py`(ICP) 는 실내에서 오탐이 잦으니 참고만.
  **이제 기준 프레임이 같아 `extrinsic_T` 와 `ref_from_body_xyz` 는 같은 값이다**
  (check_frames.py 가 이것도 검사한다).

---

## 환경 준비 (한 번만)

conda 가 시스템 파이썬을 가린다. ROS 작업 전에는 반드시:

```bash
conda deactivate                              # 또는
conda config --set auto_activate_base false   # 아예 자동 활성화를 끈다
```

`catkin_pkg` 가 없다는 빌드 에러, `rclpy._rclpy_pybind11` 를 못 찾는다는 에러가
나면 conda 파이썬을 쓰고 있는 것이다.

URDF/TF 의존성 (`description.launch.py` 가 `xacro` 명령을 실행한다. 없으면
"xacro 파일이 없다/실행 실패" 로 보인다):

```bash
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher
```

측위용 파이썬 의존성 (`global_localization.py`, `transform_fusion.py` 가 쓴다):

```bash
sudo apt install ros-humble-tf-transformations
/usr/bin/python3 -m pip install --user open3d "scipy>=1.13" "transforms3d>=0.4.2"
```

`~/.local` 에 numpy 2.x 가 있으면 apt 의 scipy 1.8 / transforms3d 는 numpy 1.x
ABI 로 빌드되어 있어 import 가 깨진다. 위처럼 numpy2 호환 버전을 --user 로
덮어써야 한다. 반드시 `/usr/bin/python3` 로 설치할 것 - conda 쪽에 깔면 ROS 가
못 찾는다.

## 라이다 네트워크

rbio 실차(TransferRobot-Hanyang 의 Jetson)와 같은 배치다: **유선 포트 하나에 라이다 하나**,
포트마다 서브넷이 다르고 host_ip 는 포트별로 하나씩 고정한다. 드라이버 설정
(`src/livox_ros_driver2/config/multi_MID360_config.json`)은 rbio 의 `MID360_config_all.json` 과
lidar_ip / host_ip / UDP 포트(라이다측 56100~56500, 호스트측 56101~56501)가 같다.
SDK 는 `host_ip` 를 글자 그대로 bind 하므로 표의 host_ip 가 이 PC 의 포트에 실제로 붙어 있어야 한다.

| 포트(NIC) | 라이다 | lidar_ip | host_ip | 비고 |
|---|---|---|---|---|
| `enxb0386cf17bd0` (USB) | livox_top | 192.168.1.135 | 192.168.1.50 | IMU 공급, 앱 LiDAR 1 |
| `enP8p1s0` (내장) | livox_front | 192.168.2.102 | 192.168.2.50 | 앱 LiDAR 2 (rbio 는 `/livox/lidar` 로 remap) |
| `enxb0386cf1873c` (USB) | livox_rear | 192.168.3.144 | 192.168.3.50 | 앱 LiDAR 3 |

`enx…` 는 USB 이더넷 어댑터의 MAC 기반 이름이라 어댑터를 바꾸면 이름이 바뀐다 (`ip -br link` 로 확인).
라이다는 자기 서브넷의 host_ip 로만 응답하므로 **케이블을 다른 포트에 꽂으면 안 붙는다**
(192.168.1.135 는 반드시 192.168.1.50 이 있는 포트에).

점검/설정은 `tools/lidar_net.sh`:

```bash
tools/lidar_net.sh status            # 포트 <-> host_ip <-> 라이다 표, 링크/ping, JSON 대조, SDK 패치 여부
sudo tools/lidar_net.sh install      # 포트마다 고정 IP (NetworkManager, 부팅 시 자동 적용)
sudo tools/lidar_net.sh install top=enxAAAA front=enP8p1s0 rear=enxBBBB   # 포트 이름이 다른 장비
```

`install` 은 포트마다 NetworkManager 연결을 `ipv4.method manual`, 해당 host_ip/24,
`ipv4.never-default yes` (유선이 기본 게이트웨이를 가져가 Wi-Fi 인터넷을 끊는 것을 막는다) 로
바꾼다. 손으로 하려면 포트마다:

```bash
nmcli -t -f NAME,DEVICE con show                      # 포트에 묶인 연결 이름 확인
sudo nmcli con mod "Wired connection 2" ipv4.method manual ipv4.addresses 192.168.2.50/24 \
  ipv4.gateway "" ipv4.dns "" ipv4.never-default yes
sudo nmcli con up "Wired connection 2"
```

드라이버가 `bind failed` -> `Init lds lidar fail!` 로 죽으면 십중팔구 host_ip 가 이 PC 에 없는 것이다
(ping 은 되는데 안 붙는 게 특징). `status` 에서 host_ip 는 있는데 ping 이 NO 면 케이블이 다른 포트에
꽂힌 것이다.

**SDK multi-NIC 패치가 필수다.** 순정 Livox-SDK2 는 detection 소켓을 첫 번째 라이다의
host_ip 하나에만 열어서, 포트(서브넷)가 3개면 라이다 1대만 붙는다. `third_party/Livox-SDK2` 에는
rbio 와 동일한 `patches/livox-sdk2-multi-nic-detection.patch` 가 적용돼 있다 (host_ip 마다 detection
소켓을 연다). SDK 를 새로 받아오면 `git apply patches/livox-sdk2-multi-nic-detection.patch` 후 빌드.

## rbio TransferRobot 스크립트로 실차 기동

rbio 저장소(`~/ros/TransferRobot`, 코드 수정 금지)의 sh 는 CAN 서비스·모터·nav 스택을
systemd 유저 유닛으로 관리한다. 설계상 라이다/측위/Nav2 는 "외부 nav 워크스페이스"가
띄우게 돼 있고, hw 가 그 역할을 한다. `tools/rbio_env.sh` 가 rbio 스크립트의 기대값
(`/home/rb/ros2_ws/src/bringup.launch.py`, ROS_DOMAIN_ID 30)을 hw 로 돌린다.

```
transfer-robot-can.service (system)   can0 up            <- rbio systemd/system
manage_motor_node.sh                  rbio motor_node    <- 쓰지 않는다 (아래 참고)
manage_navigation_stack.sh start      hw bringup.launch.py 를 유닛으로 실행
                                       = 라이다 3대 + 병합 + FAST-LIO + Nav2 + hw motor_node(/odom)
```

1. CAN 설치 (한 번): `sudo ~/hw/tools/can_setup.sh install` (아래 "CAN" 절).
   rbio 의 `install_motor_autostart.sh` 는 250 kbps 유닛을 깔므로 **둘 중 하나만** 쓴다.
2. hw 빌드 (SDK 설치 포함, 아래 "빌드" 절).
3. 기동:
   ```bash
   source ~/hw/tools/rbio_env.sh
   ~/ros/TransferRobot/scripts/manage_navigation_stack.sh start    # 45 초 안에 노드 확인
   ~/ros/TransferRobot/scripts/manage_navigation_stack.sh status
   ```
   스크립트는 `/livox_lidar_publisher`, `/laser_mapping`, `/bt_navigator` 와 `/map`,
   `/navigate_to_pose` 가 보여야 성공으로 본다. 모두 hw 가 낸다.
4. 확인 (같은 셸에서, ROS_DOMAIN_ID=30):
   ```bash
   ros2 topic hz /livox_merge/merged_pointcloud /odom
   ros2 topic echo /motor_node/initialization_status --once      # READY
   ros2 topic pub -1 /mode std_msgs/Int32 "{data: 1}"            # 애커만 주행 허용
   ```
   RViz 는 hw 의 `bringup.launch.py` 가 `rviz:=true` 로 같이 띄운다.
5. 정지: `manage_navigation_stack.sh stop`. 로그: `manage_navigation_stack.sh logs`.

모터노드는 rbio 것이 아니라 **hw motor_node** 를 쓴다 (`bringup.launch.py motor:=true` 기본).
rbio motor_node 는 `/odom` 을 내지 않아 MPPI 속도 피드백이 비고, `/mode` 도 없다.
rbio 의 `manage_motor_node.sh` 는 rbio 쪽 `install/motor_node` 가 없으면 경고만 내고
넘어가므로 **rbio 의 motor 는 빌드하지 않는다** (`build_transfer_robot.sh motor` 금지).
둘 다 뜨면 CAN 을 두 노드가 동시에 잡는다.

### rbio UI 앱까지 붙일 때 (app 모드)

앱은 경로계획을 하지 않는다. `/navigate_to_pose` 로 목표를 넣고, Nav2 가 내는
**`/cmd_vel_nav` 를 받아 상한(0.18 m/s, 0.16 rad/s) 클램프 후 `/cmd_vel` 로 중계**하며
300 ms 워치독으로 0 을 낸다. 그래서 앱과 같이 돌릴 때 Nav2 는 `/cmd_vel` 을 직접 내면 안 된다.
`bringup.launch.py app:=true` (rbio 스크립트 경유 시 `tools/rbio_env.sh` 의 `HW_APP_MODE=true`)
가 아래를 한 번에 바꾼다. 단독 테스트(`app:=false`, 기본)는 영향이 없다.

| 항목 | app:=false (단독) | app:=true (앱) |
|---|---|---|
| Nav2 속도 출력 | controller → `cmd_vel_nav` → smoother → **`/cmd_vel`** | controller → `cmd_vel_nav_raw` → smoother → **`/cmd_vel_nav`** → 앱 → `/cmd_vel` |
| motor_node 시동 후 | mode 0(정렬)에서 `/mode` 대기 | 센터링 뒤 자동 애커만 (`startup_mode 1`) |
| 도킹 속도 상한 | 전역값 | 0.04 m/s / 0.05 rad/s (rbio 도킹값) |

앱 호환 인터페이스는 항상 켜져 있다 (단독 테스트에 무해):
* `motor_node/drive_mode` 상태 토큰은 앱이 아는 이름 — `AUTONOMOUS`(애커만), `DOCKING`(디프),
  `ENTERING_DOCKING`/`EXITING_DOCKING`(전환 중), `ALIGN`(정렬 대기 = 앱은 준비 안 됨), `FAILED`.
  상세에 `[ACKERMANN]` 처럼 hw 모드명이 붙는다.
* `motor_node/set_docking_mode`(SetBool): true → 디프, false → 애커만. `/mode` 와 같은 상태머신.
* `docking/cmd_vel`: 디프 모드 전용, rbio 부호(`linear.x` + = 우측 횡이동 → 내부 `v_lat = -x`).
* `/localization`: `tf_2d.py` 가 map → 지면 평면 자세로 낸다. `child_frame_id` 는 앱 호환을
  위해 `base_footprint` 문자열을 유지하지만 TF 트리에는 그 프레임이 없다(= `base_link`).
  바꾸려면 `tf_2d` 의 `footprint_frame` 파라미터.
  transform_fusion 의 3D(map → body) 출력은 `/localization_3d` 로 옮겼다.
* behavior_server 에 `backup`, `drive_on_heading` 추가 (앱이 도킹 접근·후퇴에 직접 호출).
* `tools/rbio_env.sh` 의 `TRANSFER_ROBOT_LIDAR_*` 가 앱의 장애물/도킹 인식 좌표를 hw extrinsic 에 맞춘다.

## 빌드

Livox SDK는 colcon이 아니라 시스템에 직접 설치한다 (드라이버가 `/usr/local/lib` 를 하드코딩).

```bash
cd ~/hw/third_party/Livox-SDK2
cmake -B build && cmake --build build -j$(nproc)
sudo cmake --install build && sudo ldconfig
```

`build/` 는 `.gitignore` 의 `build/` 패턴에 걸려 git 에 들어가지 않는다 (의도된 것).
새 PC 에서는 위 명령이 `build/` 를 새로 만든다.

그 다음 워크스페이스 전체:

```bash
cd ~/hw
colcon build --symlink-install
source install/setup.bash
```

## 실행

```bash
# 구동(CAN) + 센서 + 측위 + Nav2 + RViz 한 번에
ros2 launch bringup bringup.launch.py
# 모터노드가 READY 가 되면 주행 모드를 골라준다 (기본은 0=정렬, 구동 차단)
ros2 topic echo /motor_node/initialization_status --once   # READY|...
ros2 topic pub -1 /mode std_msgs/Int32 "{data: 1}"         # 1=애커만(4WS 역위상)
```

CAN 이 없는 개발 PC 에서는 `motor:=false`. 조이스틱 수동주행이 필요하면
`ros2 launch motor_node bringup.launch.py` (모터노드 + teleop_twist_joy) 를 따로 띄우고
bringup 은 `motor:=false` 로 — 모터노드가 둘 뜨면 CAN 을 같이 잡는다.

그 다음 RViz 에서:

1. 터미널에 `Global map received` 가 뜬 뒤 **2D Pose Estimate** 로 초기 자세를
   찍는다 (일찍 찍어도 준비되면 자동 적용된다). 2초마다
   `ICP fitness: 0.9x (1.0m), 0.8x (0.4m)` 가 찍히고 `rejected` 경고가 없으면
   정합된 것. `/cur_scan_in_map` 디스플레이를 켜면 노란 스캔이 지도 위에
   수평으로 겹쳐 보인다.
2. **Nav2 Goal** 로 목표를 준다.

계층별로 나눠서:

```bash
ros2 launch bringup sensors.launch.py                       # 라이다 3대 + IMU + 병합 + TF
ros2 launch fast_lio_localization localization.launch.py    # 측위 + 2D 평면화 + 맵서버
ros2 launch navigation navigation_launch.py                 # Nav2
```

측위만 확인: `ros2 launch bringup bringup.launch.py navigation:=false`

`motor_node bringup.launch.py` 의 teleop_twist_joy 는 **같은 `/cmd_vel` 에
발행**하므로 자율주행 중에는 조이스틱 enable 을 누르지 않는다. 수동 보조:

```bash
ros2 run motor_node drive_set.py
ros2 run motor_node homing_set.py
ros2 run motor_node state_monitor.py
```

### CAN (Jetson mttcan)

```bash
sudo tools/can_setup.sh up                 # 모듈(can, can_raw, can_dev, mttcan) + can0 설정 + up
tools/can_setup.sh status                  # 링크/카운터, candump 2초 수신 (TPDO 0x181~0x184)
sudo tools/can_setup.sh install            # 부팅 자동 실행: /etc/systemd/system/can0_setup.service
sudo tools/can_setup.sh install --bitrate 250000   # 드라이버가 250 kbps 면
```

기본 1 Mbps (`drive_set.py` 와 동일), `restart-ms 100` (bus-off 자동 복구), `txqueuelen 1000`.
rbio 의 `transfer-robot-can.service` 는 250 kbps 다. `can0` 장치 자체가 없으면 `jetson-io` 로
CAN 핀먹스를 켜야 한다.

**Jetson 은 외장 트랜시버가 필요하다.** `c310000.mttcan` 핀은 3.3V TTL 이라 SN65HVD230 /
TCAN1042 같은 트랜시버를 거쳐야 버스에 붙는다. 트랜시버 STB/EN 이 뜬 상태면 통신이 안 된다.

#### 통신이 안 될 때

```bash
tools/can_setup.sh errors                  # 에러프레임 "종류" 확인 (가장 유용)
sudo tools/can_setup.sh selftest           # 루프백 자체진단 (버스 케이블 분리하고)
sudo tools/can_setup.sh listen             # listen-only 로 엿듣기 (ACK/에러프레임 안 냄)
```

`status` 읽는 법:

| 증상 | 해석 |
|---|---|
| `berr-counter` 0, 수신 없음 | 버스가 조용하다. 드라이버 전원/배선을 본다 |
| `rx` 카운터가 순식간에 127 → `ERROR-PASSIVE`, 유효 프레임 0 | 라인에 신호는 있는데 디코드 불가. 비트레이트를 바꿔도 같으면 **버스 도미넌트 고착**(CAN_H/L 단락, 트랜시버 고착, 배선 오류) 쪽이다 |
| `RX packets` 가 `error-warn`+`error-pass` 개수와 같다 | 그 패킷은 실제 데이터가 아니라 상태변화 에러프레임이다 (유효 수신 0) |
| `ack` 에러만 난다 | 우리는 송신하는데 응답할 노드가 없다 |

`errors` 의 에러프레임 종류로 갈린다: `stuff`/`form`/`crc` → 비트레이트·노이즈·종단저항,
`bit0`/`bit1` → 배선/트랜시버, `ack` → 상대 노드 없음.

배선 확인 (전원 끄고 멀티미터): CAN_H–CAN_L 저항이 **약 60 Ω** 이어야 한다 (양끝 120 Ω 병렬).
120 Ω 이면 종단이 한쪽뿐, 수백 Ω 이상이면 종단 없음, 0 Ω 에 가까우면 단락이다.
전원 켠 상태에서 GND 기준 전압은 유휴 시 CAN_H·CAN_L 둘 다 약 2.5 V 여야 한다.
계속 3.5 V / 1.5 V 로 갈려 있으면 도미넌트 고착이다.

## motor_node (하부 구동)

rbio TransferRobot motor_node 구조를 따른다: 주행 ID 1/3 은 Profile Velocity, 조향 ID 2/4 는
Profile Position(0x607A) + New set-point/ACK 핸드셰이크. 시동 시 Fault reset 후 조향을 0 도로
센터링하고 **mode 0(정렬)** 에서 대기한다. 조향 감속비 10:1, 엔코더 131072 pulse/rev,
직진 = 0x6064 0 (rbio 기준).

| `/mode` (Int32) | 모드 | 조향 | cmd_vel 해석 |
|---|---|---|---|
| 0 | ALIGN 정렬 | 앞뒤 0° 유지 | 무시, 구동 차단 |
| 1 | ACKERMANN | 4WS 역위상 (뒤 = −앞) | `linear.x`, `angular.z` |
| 2 | DIFF 디프 | 앞 −`diff_steer_deg`(−90°) / 뒤 +`diff_steer_deg`(90°) 고정 | `linear.x`(또는 `y`) = 횡이동(+좌), `angular.z` = 제자리 회전 |
| 3 | JOY 수동 | 4WS 역위상 (애커만과 같은 바퀴 방향), **각도가 아니라 각속도 지령** | `linear.x` = 주행 속도, `angular.z` = 조향 각속도 |

JOY 모드만 조향축(ID 2/4)을 **Profile Velocity(0x6060=3, RPDO→0x60FF)** 로 바꿔 쓴다.
진입할 때 조향을 0 도로 센터링한 뒤 전환하고, 빠져나갈 때(모드 변경·`initialize`·fault
복구·전환 실패) Profile Position 으로 되돌린다. 조이스틱은 `teleop_twist_joy` 가 내는
`/cmd_vel` 을 그대로 쓰고, 해석만 이 모드에서 달라진다.

* 스틱은 `max_angular_vel` 로 정규화한다 — 풀 스케일에서 `joy_steer_rate_deg_s`(기본 20°/s).
* `max_steering_angle_deg`(기본 55°)에 닿으면 **더 밀어넣는 방향만** 0 으로 막는다.
* 구동은 앞뒤 같은 부호·같은 속도. 수동 상한은 `joy_max_linear_vel`(0 이면 전역값).
* PV 는 새 지령을 안 주면 마지막 속도로 계속 돈다. cmd_vel 타임아웃·모터 stale·fault·
  모드 이탈에서 전부 속도 0 을 먼저 보낸다.

모드 전환은 **주행이 멈춘 뒤** 조향을 목표각으로 옮기고 3° 이내 도달하면 활성화된다
(`motor_node/drive_mode` 에 `ALIGN|…`, `TO_DIFF|…`, `DIFF|…`). 애커만 진입 직후에는
중립 cmd_vel 을 한 번 받아야 구동이 풀린다 (Nav2/조이스틱이 정지 명령을 보내므로 보통 자동).

**오도메트리는 모드와 무관한 단일 모델**이다. 지령이 아니라 측정된 앞/뒤 조향각과
앞/뒤 휠속도로 몸체 속도를 역산한다 (`v_f`, `v_r`: 접선속도, `δf`, `δr`: 조향각, L = 1.29):

```
vx = (v_f cosδf + v_r cosδr)/2,  vy = (v_f sinδf + v_r sinδr)/2,  wz = (v_f sinδf − v_r sinδr)/L
```

그래서 90° 디프 모드의 횡이동/제자리 회전, 전환 중, 90° 가 정확히 안 나온 경우에도 끊기지
않고 맞는다. 피드백이 stale 이면 적분을 멈추고 공분산을 1e6 으로 키운다.
`/odom` 은 twist(MPPI 속도 피드백)만 Nav2 가 쓰고, `odom->base_link` TF 는 tf_2d.py 가
내므로 `publish_odom_tf` 는 false 로 둔다.

토픽/서비스: `/mode`, `/cmd_vel`, `/odom`, `/steer_angle_deg`[전실제,후실제,전지령,후지령],
`motor_node/{command_ack, diagnostics, initialization_status, drive_mode}`,
`motor_node/initialize`(재초기화·센터링), `motor_node/reset_odom`,
rbio 앱 호환 `motor_node/set_docking_mode`, `docking/cmd_vel`. 파라미터는
`src/motor_node/launch/motor.launch.py` (`startup_mode` 는 app 모드에서 1,
JOY 관련은 `joy_steer_rate_deg_s` / `joy_max_linear_vel` / `joy_stick_deadzone`).

## lift_node (리프트)

rbio TransferRobot-Hanyang 앱의 `MotorControlManager` 리프트 부분을 ROS2 노드로 옮긴 것.
리프트 제어보드와 **RS232 직결** (`/dev/ttyTHS1`, 115200 8N1 — 앱 `run_transfer_robot.sh` 기본값).
프레임은 `[0x3E][프로토콜][길이][페이로드][CRC-8 poly 0x07]`, 리프트는 페이로드 1바이트.

| 토픽 (Int32) | 프로토콜 | 0 | 1 | 2 |
|---|---|---|---|---|
| `lift/vertical` | 0x10 | 정지 | 상승 UP | 하강 DOWN |
| `lift/horizontal` | 0x11 | 정지 | 전진 EXTEND | 후진 RETRACT |

* 보드는 **마지막 지령을 래치**한다 (앱은 버튼 누름에 1/2, 뗌에 0 을 한 번씩 보냈다). 그래서
  노드는 `cmd_timeout_s`(0.5 s) 안에 같은 지령이 다시 안 오면 정지를 보낸다. 움직이려면
  `ros2 topic pub -r 10` 처럼 **계속 발행**해야 하고, 발행이 끊기면 선다. 같은 값이 반복되면
  보드에는 다시 안 보내고 타임아웃만 연장한다. `cmd_timeout_s: 0.0` 이면 앱처럼 래치.
* 정지 프레임은 75/180 ms 뒤 두 번 더 보낸다 (앱과 동일). 포트 열림·재연결·종료·
  `lift_node/stop` 은 리프트 2축 + 호이스트 4개(0x20) 를 한꺼번에 정지한다 (앱 `stopAll` 묶음).
* 연결 직후 0x40(로봇암 오류 조회) 을 보내 왕복 응답으로 링크를 점검한다 (앱 startup probe).
  응답이 없으면 `NO_RESPONSE` 로 두되 지령은 막지 않는다 (진단 WARN).
* 포트 오류는 2 s 간격으로 다시 연다. 사용자가 `dialout` 그룹이어야 한다:
  `sudo usermod -aG dialout $USER` 후 재로그인. Jetson `nvgetty` 가 ttyTHS1 을 잡고 있으면
  `sudo systemctl disable --now nvgetty`.

```
ros2 launch lift_node lift.launch.py                                 # 단독 (bringup 은 lift:=true 기본)
ros2 topic echo /lift_node/status --once                             # READY|probe … / NO_RESPONSE|…
ros2 run lift_node lift_action.py up                                 # 액션: 전 구간 상승 후 정지
ros2 run lift_node lift_action.py down -d 2.5 --feedback             # 2.5 초 하강, 진행 상황 출력
ros2 run lift_node lift_teleop_key.py                                # 키보드: 누르는 동안 구동, 떼면 정지
ros2 topic pub -r 10 /lift/vertical std_msgs/msg/Int32 "{data: 1}"   # 상승. Ctrl-C 하면 0.5 s 뒤 정지
ros2 service call /lift_node/stop std_srvs/srv/Trigger
ros2 run lift_node lift_set.py probe                                 # ROS 없이 보드 직접 점검 (노드는 내리고)
ros2 run lift_node lift_set.py up --hold 1.0                         # 1 초 상승 후 정지
```

**액션** `lift_node/move` (`lift_node/action/LiftMove`): 한 축을 정해진 시간만큼 구동하고
끝나면 정지까지 보낸다. goal 은 `axis`(0 수직/1 수평), `direction`(1 상승·전진/2 하강·후진),
`duration`(0 이면 노드 파라미터의 방향별 전 구간 시간). 피드백은 `elapsed`/`remaining`,
결과는 `success` 와 `DONE|…`/`CANCELED|…`/`ABORTED|…` 메시지다.

* 보드가 **위치·리밋을 알려주지 않으므로** "동작이 끝났다" 의 기준은 **구동 시간**이다.
  보드가 되돌리는 프레임(RX)은 받은 지령의 에코일 뿐 완료 신호가 아니다. 기본값은 벤치에서
  잰 전 구간 시간이다 — 상승은 지령부터 최대점에서 멈출 때까지 **약 4.2 s**
  (`vertical_up_duration_s`). `vertical_down_duration_s` / `horizontal_extend_duration_s` /
  `horizontal_retract_duration_s` 도 같은 식이고, 상한은 `max_move_duration_s`(30 s).
  실제 스트로크를 다시 재면 이 파라미터만 고치면 된다.
* goal 이 사는 동안 서버가 20 ms 마다 지령을 갱신해 `cmd_timeout_s` 로 서지 않게 한다.
  시간이 다 되면 정지 프레임(+75/180 ms 재전송) 을 보내고 succeed 한다.
* 동시에 사는 goal 은 하나다. **취소**, **새 goal 의 선점**, 그 축에 들어온 **수동 토픽 지령**,
  `lift_node/stop`·`reconnect`, **링크 끊김** 어느 쪽으로 끝나든 goal 은 결과를 돌려받고
  축은 선다 (수동 지령에 밀린 경우는 그 지령이 축을 덮어쓴다).
* CLI `lift_action.py up|down|extend|retract [-d 초] [--feedback] [--no-wait]` — Ctrl-C 하면
  goal 을 취소하고 서버가 축을 세운 뒤 돌려주는 결과까지 받는다.
  종료 코드 0 성공 / 1 실패(거절·중단·취소) / 2 서버 없음.

**키보드 텔레옵** `lift_teleop_key.py`: ↑/w 상승, ↓/s 하강, →/d 전진, ←/a 후진, space 전부 정지,
q 종료. 터미널은 키 뗌 이벤트가 없어서 **OS 자동반복 문자가 끊기는 것**을 뗀 것으로 본다 —
첫 누름 뒤 반복이 시작될 때까지는 `--tap-timeout`(0.6 s), 반복 중에는 `--release-timeout`(0.15 s)
안에 문자가 없으면 정지. 짧게 톡 치면 최대 0.6 s 뒤에 서고, 누르고 있다 떼면 ~0.15 s 뒤에 선다.
시작할 때 움찔거리면(OS 반복 지연 > 0.6 s) `--tap-timeout` 을 늘린다. SSH 로 쓸 때 반복은
**접속한 쪽 PC** 설정을 따른다 (X11: `xset r rate 200 40`).

토픽/서비스/액션: `lift/vertical`, `lift/horizontal`,
`lift_node/{command_ack, status(latched "STATE|detail"), diagnostics, rx(수신 hex)}`,
`lift_node/{stop, reconnect, probe}`(Trigger), `lift_node/move`(LiftMove 액션). 파라미터는 `src/lift_node/launch/lift.launch.py`.
호이스트(0x20)·로봇암(0x30~0x51) 구동은 아직 안 옮겼다 — 정지 프레임만 낸다.

## Nav2 구성

| 항목 | 값 | 이유 |
|---|---|---|
| 플래너 | `SmacPlannerHybrid`, DUBIN, `minimum_turning_radius` 1.30 | 조향식 차량. 2D 플래너는 회전반경 없는 경로를 내서 못 쓴다 |
| 컨트롤러 | `MPPIController`, `motion_model: Ackermann`, `min_turning_r` 1.30 | 롤아웃 비용으로 동적 장애물을 직접 회피. RPP 는 회피를 못 한다 |
| 장애물 입력 | `/livox_merge/merged_pointcloud_sliced` -> local/global `obstacle_layer` | 지면 위 0.15~2.3 m 슬라이스 + 차체 XY 크롭(`slice_crop_half_*`) |
| local costmap | 12x12 m, 10 Hz | 기본 5x5 m 는 범퍼 앞 1.7 m 라 동적 회피가 성립 안 함. `width`/`height` 는 **정수**로 |
| MPPI 지평선 | `time_steps` 130 x `model_dt` 0.1 = 13 s (2.6 m) | 차체 1.62 m + 90도 선회호 2.04 m 를 담아야 회피 궤적이 나온다 |
| 속도 한계 | vx 0.20, wz 0.30 (MPPI, velocity_smoother, behavior_server) | motor_node 의 `max_linear_vel` / `max_angular_vel` 와 동일해야 한다 |

`cmd_vel` 의 (v, ω) 는 차량 중심(base_link) 속도이고 조향각 변환은 motor_node
가 한다. 실제 최소회전반경은 (축간 1.29/2)/tan55° ≈ 0.45 m 라 1.30 은 보수값이다.

**시작 위치 주의:** Smac 은 시작 footprint(0.81x0.48, local/global 동일) 가 치명 셀과 겹치면
`Starting point in lethal space` 로 계획을 거부한다. 벽에 붙여 주차한 상태에서
goal 을 주면 이 때문에 안 움직이니, 벽에서 20~30 cm 이상 떨어진 곳에서 시작한다.

## 맵 만들기

**매핑 패키지는 이 워크스페이스에 없다.** `fast_lio`(매핑)와 `pcd2pgm`(3D->2D
변환)은 주행에 쓰이지 않아 제거했다. 맵을 새로 만들어야 하면 두 패키지를 다시
받아서 별도 워크스페이스에서 돌리고, 결과물만 `src/navigation/map/` 으로 옮긴다.

현재 맵은 `test.pcd`(3D, 측위용) + `test.pgm`/`test.yaml`(2D, 코스트맵용) 이다.
`.yaml` 의 `image:` 는 반드시 상대경로로 둘 것. `/map_save` 서비스는 이 경로에
덮어쓰므로 측위 중에는 부르지 않는다.

맵 교체는 **launch 인자 하나**로 한다. 파일을 `navigation/map/` 에 `<이름>.yaml`
(+ `.pgm`) 과 `<이름>.pcd` 짝으로 넣고:

```bash
ros2 launch bringup bringup.launch.py map:=<이름>          # 실장비
ros2 launch bringup bag_localization.launch.py map:=<이름> # bag 재생
```

`map` 인자가 2D(`<이름>.yaml` -> map_server) 와 3D(`<이름>.pcd` ->
fast_lio_mapping / global_localization / global_map_publisher) 경로를 함께 만든다.
`mid360.yaml` 과 `nav2_params.yaml` 에는 맵 경로가 없다 - 두 곳에 값이 남아 있으면
인자를 바꿔도 조용히 옛 맵을 읽기 때문에 아예 지웠다.

**새 파일은 넣은 뒤 `colcon build` 를 한 번 해야 한다.** `install/.../share/navigation/map/`
는 파일별 심볼릭 링크라, 빌드하지 않으면 새 맵이 share 에 없어서
`3D prior map 이 없다: ...` 에러가 난다.

## bag 재생 테스트

```bash
ros2 launch bringup bag_localization.launch.py            # 측위. 기본 bag 은 rosbag2_differential, bag:= 로 변경
ros2 launch navigation navigation_launch.py use_sim_time:=true   # Nav2 까지 보려면
```

* bag 은 xfer_format=0 으로 기록되어 있고 실장비도 지금은 같은 형식이라
  경로가 하나다(`src/bringup/params/livox_merge_bag.yaml`). 다른 건 `clouds`
  뿐이다 - 기존 bag 은 front/rear 만 녹화돼 있어 2 이고, top 까지 들어 있는
  bag 이면 `clouds:=3` 으로 띄운다.
* 초기 자세: RViz 2D Pose Estimate, 또는 이 bag 은 지도 원점 근처에서 시작하므로
  `ros2 topic pub -1 -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}'`
* goal 은 `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...`
  로 준다. `ros2 topic pub -1 /goal_pose` 는 디스커버리 레이스로 씹힌다.
* bag 로봇은 녹화된 궤적대로만 움직인다(open-loop). cmd_vel 은 나오지만 차가
  goal 을 따라가지는 않으므로, 코스트맵 마킹 / 경로 / `/cmd_vel` 주기(10Hz)와
  회전반경 제약까지만 확인할 수 있다. 재생이 끝나면 `/clock` 이 멈춰 Nav2 가
  반응하지 않는다 - 다시 하려면 launch 를 재시작한다.
* 센서 토픽은 Best Effort 라 RViz PointCloud2 디스플레이의 Reliability 를
  Best Effort 로 둬야 보인다 (기본 rviz 설정에 들어 있음).

## 확인

```bash
python3 tools/check_frames.py                            # URDF <-> 설정 정합 (값 바꿀 때마다)
python3 tools/check_lidar_z.py                           # 장착 z 가 실물과 맞는가 (천장 평면 기준)
rviz2 -d src/navigation/rviz/frames_check.rviz           # 모델과 클라우드 겹쳐보기
ros2 run tf2_ros tf2_echo map base_link                  # z = 0(지면), camera_init 은 없어야 정상
ros2 run tf2_tools view_frames
ros2 topic hz /livox_merge/merged_pointcloud_sliced      # 10 Hz
```

측위가 이상해 보이면 순서대로: (1) initialpose 를 줬는가 (안 주면 스캔이
세로로 서고 경로가 뭉개진다) (2) `ICP fitness` 가 0.8 이상인가 (3)
`rejected: vehicle roll/pitch` 경고가 있는가 - 있으면 ref_from_body 나 URDF 가
틀린 것이니 check_frames 부터.

## 아직 미정 / 실장비에서 확인할 것

- `mount.xacro` 의 `top_xyz` z 값(livox_frame 기준 1.0265)이 실측 전 추정치다.
  오차 +-0.025.
- 최소회전반경 1.30 m 는 보수값. 현장에서 안정 확인 후 0.7~0.8 로 낮출 수 있다
  (플래너 `minimum_turning_radius` 와 MPPI `min_turning_r` 를 같이).
- MPPI `batch_size` 1200 은 데스크톱 기준. Jetson 에서 `Control loop missed`
  경고가 나면 줄인다.
- front/rear 라이다 프레임 시작 시각이 동기화되지 않아 한쪽 점들의 시각이 최대
  100 ms 어긋날 수 있다. 저속이라 영향은 작다.
- 벽에 붙은 시작 위치에서 계획 거부 (위 Nav2 구성 참조). 2D 맵의 벽 두께/잡음을
  줄이는 재생성 도구가 있으면 완화된다.
