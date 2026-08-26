# hw — 4WS(후륜 역조향) 플랫폼

Livox MID-360 3대 + 내장 IMU로 **FAST-LIO 3D 측위**를 하고, 그 자세를
**평면으로 눌러서(z/roll/pitch 제거)** **Nav2 2D 내비게이션**에 넘기는 차량
한 대분의 전체 설정. 차량은 앞/뒤 축을 반대로 꺾는 **4WS(후륜 역조향)** 이고
Nav2 는 MPPI(Ackermann 모델) + Smac Hybrid-A* 로 돌린다.

```
hw/
├── src/                            colcon 워크스페이스 (여기 하나뿐)
│   ├── livox_ros_driver2/          MID-360 x3 드라이버 (CustomMsg)
│   ├── livox_merge/                front/rear 2대를 하나로 병합 -> PointCloud2
│   ├── fast_lio_localization/      FAST-LIO 측위 + ICP 전역정합 + tf_2d 평면화
│   ├── motor_node/                 CAN 4WS 구동 (cmd_vel -> 조향/구동, /odom)
│   ├── navigation/                 ★ URDF/TF · Nav2 파라미터 · behavior tree · 맵
│   └── bringup/                    ★ 전체 launch, bag 재생 테스트
├── third_party/Livox-SDK2/         colcon 대상 아님. sudo make install 필요
├── tools/                          check_frames.py, ply_to_pcd.py
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
MID-360 front ─┐  CustomMsg
MID-360 rear  ─┴─ livox_merge ─┬─ /livox_merge/merged_pointcloud         (전체)  ─┐
                               └─ /livox_merge/merged_pointcloud_sliced  (z 슬라이스 + 차체 크롭) ─ Nav2 costmap
MID-360 top ───── /livox/imu_192_168_1_135 ────────────────────────────────────────┤
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
map ── odom ── base_link ── base_footprint ── livox_frame ─┬─ livox_top ── imu_link
               (tf_2d)      (지면)            (라이다 중심) ├─ livox_front
                                                           └─ livox_rear
```

* `map->odom`, `odom->base_link` 는 `tf_2d.py` 가 낸다. `base_link` 는 z 를 0 으로
  누른 평면 프레임이고, `base_footprint` 는 그 0.5 m 아래 **지면**(REP-120)이다.
  차량 모델과 센서는 전부 `base_footprint` 아래에 있다.
* FAST-LIO 의 `camera_init` / `body` 는 **TF 에 올리지 않는다.** `body` 는 차량
  기준점이 아니라 상단 라이다 안에서 옆으로 누운 **IMU 프레임**이라, 그대로
  평면화하면 yaw 가 엉킨다. body -> base_link 변환은
  `fast_lio_localization/config/mid360.yaml` 의 `ref_from_body_*` 가 들고 있고
  `tf_2d.py` 와 `global_localization.py` 가 파라미터로 받는다.
  RViz 에서 camera_init 프레임 토픽(`/cloud_registered` 등)을 보고 싶을 때만
  `localization.launch.py debug_tf:=true`.
* 같은 기하가 세 곳에 있다: `mount.xacro`(원본), `livox_merge` 의 extrinsic
  (base_footprint 기준), `mid360.yaml` 의 `extrinsic_T/R`(base_footprint -> IMU)
  와 `ref_from_body_*`(IMU -> base_link). 센서 위치를 바꾸면 `mount.xacro` 만
  고친 뒤 `python3 tools/check_frames.py` 로 나머지와의 정합을 확인한다.
  **extrinsic_T 와 ref_from_body 는 기준 프레임이 달라 같은 값이 아니다.**

---

## 환경 준비 (한 번만)

conda 가 시스템 파이썬을 가린다. ROS 작업 전에는 반드시:

```bash
conda deactivate                              # 또는
conda config --set auto_activate_base false   # 아예 자동 활성화를 끈다
```

`catkin_pkg` 가 없다는 빌드 에러, `rclpy._rclpy_pybind11` 를 못 찾는다는 에러가
나면 conda 파이썬을 쓰고 있는 것이다.

측위용 파이썬 의존성 (`global_localization.py`, `transform_fusion.py` 가 쓴다):

```bash
sudo apt install ros-humble-tf-transformations
/usr/bin/python3 -m pip install --user open3d "scipy>=1.13" "transforms3d>=0.4.2"
```

`~/.local` 에 numpy 2.x 가 있으면 apt 의 scipy 1.8 / transforms3d 는 numpy 1.x
ABI 로 빌드되어 있어 import 가 깨진다. 위처럼 numpy2 호환 버전을 --user 로
덮어써야 한다. 반드시 `/usr/bin/python3` 로 설치할 것 - conda 쪽에 깔면 ROS 가
못 찾는다.

## 빌드

Livox SDK는 colcon이 아니라 시스템에 직접 설치한다 (드라이버가 `/usr/local/lib` 를 하드코딩).

```bash
cd ~/hw/third_party/Livox-SDK2/build
cmake .. && make -j$(nproc)
sudo make install && sudo ldconfig
```

그 다음 워크스페이스 전체:

```bash
cd ~/hw
colcon build --symlink-install
source install/setup.bash
```

## 실행

```bash
# 터미널 1: 구동 (bringup.launch.py 에 포함되어 있지 않다)
ros2 launch motor_node bringup.launch.py
# 터미널 2: 센서 + 측위 + Nav2 + RViz
ros2 launch bringup bringup.launch.py
```

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

motor_node 의 launch 는 teleop_twist_joy 도 같이 띄우며 **같은 `/cmd_vel` 에
발행**하므로 자율주행 중에는 조이스틱 enable 을 누르지 않는다. 수동 보조:

```bash
ros2 run motor_node drive_set.py
ros2 run motor_node homing_set.py
ros2 run motor_node state_monitor.py
cat ~/hw/src/motor_node/script/can_guide.txt              # can0 설정
```

## Nav2 구성

| 항목 | 값 | 이유 |
|---|---|---|
| 플래너 | `SmacPlannerHybrid`, DUBIN, `minimum_turning_radius` 1.30 | 조향식 차량. 2D 플래너는 회전반경 없는 경로를 내서 못 쓴다 |
| 컨트롤러 | `MPPIController`, `motion_model: Ackermann`, `min_turning_r` 1.30 | 롤아웃 비용으로 동적 장애물을 직접 회피. RPP 는 회피를 못 한다 |
| 장애물 입력 | `/livox_merge/merged_pointcloud_sliced` -> local/global `obstacle_layer` | 지면 위 0.15~2.3 m 슬라이스 + 차체 XY 크롭(`slice_crop_half_*`) |
| 속도 한계 | vx 0.20, wz 0.30 (MPPI, velocity_smoother, behavior_server) | motor_node 의 `max_linear_vel` / `max_angular_vel` 와 동일해야 한다 |

`cmd_vel` 의 (v, ω) 는 차량 중심(base_link) 속도이고 조향각 변환은 motor_node
가 한다. 실제 최소회전반경은 (축간 1.29/2)/tan55° ≈ 0.45 m 라 1.30 은 보수값이다.

**시작 위치 주의:** Smac 은 시작 footprint(0.77x0.44) 가 치명 셀과 겹치면
`Starting point in lethal space` 로 계획을 거부한다. 벽에 붙여 주차한 상태에서
goal 을 주면 이 때문에 안 움직이니, 벽에서 20~30 cm 이상 떨어진 곳에서 시작한다.

## 맵 만들기

**매핑 패키지는 이 워크스페이스에 없다.** `fast_lio`(매핑)와 `pcd2pgm`(3D->2D
변환)은 주행에 쓰이지 않아 제거했다. 맵을 새로 만들어야 하면 두 패키지를 다시
받아서 별도 워크스페이스에서 돌리고, 결과물만 `src/navigation/map/` 으로 옮긴다.

현재 맵은 `test.pcd`(3D, 측위용) + `test.pgm`/`test.yaml`(2D, 코스트맵용) 이다.
`.yaml` 의 `image:` 는 반드시 상대경로로 둘 것. `/map_save` 서비스는 이 경로에
덮어쓰므로 측위 중에는 부르지 않는다.

맵을 바꾸면 이 네 곳을 같이 고쳐야 한다.

  fast_lio_localization/launch/localization.launch.py   2D 맵 기본값
  fast_lio_localization/config/mid360.yaml              map_file_path (3D)
  navigation/params/nav2_params.yaml                    map_server yaml_filename
  navigation/map/                                       파일 자체

## bag 재생 테스트

```bash
ros2 launch bringup bag_localization.launch.py            # 측위 (155초 재생)
ros2 launch navigation navigation_launch.py use_sim_time:=true   # Nav2 까지 보려면
```

* bag 은 xfer_format=0 으로 기록되어 라이다가 PointCloud2 라, livox_merge 를
  `input_type:=pointcloud2` 로 띄운다(`src/bringup/params/livox_merge_bag.yaml`).
  실장비는 CustomMsg 경로다. 두 경로는 같은 형식으로 정규화되지만 실장비
  경로는 bag 으로 검증되지 않는다.
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
rviz2 -d src/navigation/rviz/frames_check.rviz           # 모델과 클라우드 겹쳐보기
ros2 run tf2_ros tf2_echo map base_footprint             # z = -0.5, camera_init 은 없어야 정상
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
