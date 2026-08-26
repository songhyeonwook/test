# hw — 자율주행 차량 PC

Livox MID-360 3대 + 내장 IMU로 **FAST-LIO 3D 측위**를 하고, 그 자세를
**평면으로 눌러서(z/roll/pitch 제거)** **Nav2 2D 내비게이션**에 넘기는 차량
한 대분의 전체 설정.

이전에는 `ros2_ws` / `ws_livox` 두 워크스페이스에 흩어져 있었고 nav2 는 62M짜리
소스 트리를 통째로 들고 있었다. 지금은 **워크스페이스 하나**다.

```
hw/
├── src/                            colcon 워크스페이스 (여기 하나뿐)
│   ├── livox_ros_driver2/          MID-360 x3 드라이버
│   ├── livox_merge/                front/rear 2대를 하나로 병합
│   ├── fast_lio/                   매핑 (맵 만들 때만)
│   ├── fast_lio_localization/      측위 + tf_2d 평면화
│   ├── pcd2pgm/                    3D PCD -> 2D 점유맵
│   ├── motor_node/                 CAN 구동
│   ├── nav/                        (묶음 폴더, 패키지 아님)
│   │   ├── description/            ★ 차량/센서 형상 URDF. TF 의 유일한 출처
│   │   └── navigation/             ★ Nav2 파라미터 · behavior tree · launch
│   └── bringup/                    ★ 전체 launch + 맵
├── third_party/Livox-SDK2/         colcon 대상 아님. sudo make install 필요
├── tools/check_frames.py           URDF <-> 설정 정합 검사
├── bag/                            rosbag 기록 위치 (git 제외)
└── src_rbio/                       참고용 사본 (COLCON_IGNORE, git 제외)
```

**Nav2 본체는 소스가 아니라 apt(`/opt/ros/humble`)에서 온다.** 30개 패키지가
거기 다 있다. `src/nav/navigation` 은 이 차량에 맞춰 실제로 손댄 것만 들고 있다.

맵은 `src/bringup/maps/` 에 있다. 3D(`output.pcd`, 측위용)와 2D(`scans_new.pgm`,
코스트맵용)를 같은 폴더에 두는 이유는 **둘이 같은 주행에서 나온 짝이어야** 하기
때문이다. 어긋나면 로봇이 벽 속에 있다고 나온다.

---

## 데이터 흐름

```
MID-360 front ─┐
MID-360 rear  ─┴─ livox_merge ─ /livox_merge/merged_pointcloud ─┐
MID-360 top ───── /livox/imu_192_168_1_135 ─────────────────────┤
                                                                   │
                                                    fast_lio_localization
                                                                   │
                                          map ─ camera_init ─ body │  (3D)
                                                                   │
                                                   tf_2d.py  ← z/roll/pitch 제거
                                                                   │
                                          map ─ odom ─ base_link   │  (2D)
                                                                   │
                                                                 Nav2
                                                                   │
                                                   /cmd_vel ─ motor_node
```

TF 트리:

```
map ─┬─ odom ── base_link ── base_footprint ─┬─ livox_frame
     │                                       ├─ livox_top ── imu_link
     │                                       ├─ livox_front
     │                                       └─ livox_rear
     │
     └─ camera_init ── body ── fastlio_ref       <- FAST-LIO 내부 배관
```

**윗줄이 실제로 쓰는 체인이다.** 차량 모델과 센서는 전부 `base_footprint` 아래에
있고, `map->odom->base_link` 는 `tf_2d.py` 가 낸다.

아랫줄은 FAST-LIO 가 소유하는 프레임이라 없앨 수 없다. `body` 는 차량 기준점이
아니라 **IMU 프레임**이고(`publish_frame_body` 가 포인트를 IMU 좌표로 옮겨서
`body` 로 stamp), 그 IMU 는 상단 라이다 안에서 옆으로 누워 있다. 그대로
평면화하면 yaw 가 엉키므로 `body` 아래에 차량 정렬된 `fastlio_ref` 를 하나 두고
`tf_2d.py` 가 그걸 평면화한다. 그 외에는 볼 일이 없다.

  * `src/nav/description/urdf/vehicle.urdf.xacro`     윗줄 (RViz 가 보는 모델)
  * `src/nav/description/urdf/fastlio_ref.urdf.xacro` 아랫줄 (프레임 하나뿐)
  * `src/nav/description/urdf/mount.xacro`            장착값 원본, 둘이 같이 읽는다

센서 위치를 바꾸면 `mount.xacro` 만 고치고 `python3 tools/check_frames.py` 로
livox_merge / fast_lio 설정과의 정합을 확인한다.

---

## 환경 준비 (한 번만)

conda 가 시스템 파이썬을 가린다. ROS 작업 전에는 반드시:

```bash
conda deactivate                              # 또는
conda config --set auto_activate_base false   # 아예 자동 활성화를 끈다
```

`catkin_pkg` 가 없다는 빌드 에러가 나면 conda 파이썬을 쓰고 있는 것이다.

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

전부 한 번에:

```bash
ros2 launch bringup bringup.launch.py
```

계층별로 나눠서:

```bash
ros2 launch bringup sensors.launch.py                       # 라이다 3대 + IMU + 병합 + TF
ros2 launch fast_lio_localization localization.launch.py    # 측위 + 2D 평면화 + 맵서버
ros2 launch navigation navigation_launch.py                 # Nav2
```

측위만 확인: `ros2 launch bringup bringup.launch.py navigation:=false`

구동/수동주행:

```bash
ros2 launch motor_node bringup.launch.py
ros2 run motor_node drive_set.py
ros2 run motor_node state_monitor.py
~/hw/src/motor_node/script/can.sh
```

## 맵 만들기

```bash
ros2 launch bringup sensors.launch.py
ros2 launch fast_lio mapping.launch.py        # -> src/fast_lio/PCD/scans.pcd
ros2 run pcd2pgm pcd2pgm                      # 3D PCD -> 2D pgm
```

결과물을 `src/bringup/maps/` 에 넣고 `colcon build --packages-select bringup` 으로
다시 설치한다. `.yaml` 의 `image:` 는 반드시 상대경로로 둘 것.

3D(`output.pcd`)와 2D(`scans_new.pgm`)는 **같은 주행에서 나온 짝이어야 한다** —
측위는 3D 를, Nav2 코스트맵은 2D 를 쓰기 때문에 어긋나면 로봇이 벽 속에 있다고
나온다.

## bag 재생 측위 테스트

```bash
ros2 launch bringup bag_localization.launch.py rate:=0.3
```

재생 중 RViz 의 "2D Pose Estimate" 로 초기 자세를 찍어야 측위가 붙는다.
bag 은 xfer_format=0 으로 기록되어 라이다가 PointCloud2 라, livox_merge 를
`input_type:=pointcloud2` 로 띄운다(`src/bringup/params/livox_merge_bag.yaml`).

## 확인

```bash
python3 tools/check_frames.py                            # URDF <-> 설정 정합
rviz2 -d src/nav/description/rviz/frames_check.rviz      # 모델과 클라우드 겹쳐보기
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
```

## 아직 미정

- `src/nav/description/urdf/mount.xacro` 의 `top_xyz` z 값(0.8465)이 실측 전
  추정치다. 오차 +-0.025.
- Nav2 코스트맵이 `static_layer` 만 쓴다. 라이브 장애물 레이어를 붙이려면
  `sudo apt install ros-humble-pointcloud-to-laserscan` 후
  `/livox_merge/merged_pointcloud_sliced` 를 `/scan` 으로 변환해 넣는다.
- 병합 출력이 2.2 Hz 로 입력(9.5 Hz)보다 낮다. FastDDS 공유메모리 오류가
  같이 찍히는데 아직 원인을 못 잡았다.
