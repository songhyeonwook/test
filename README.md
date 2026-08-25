# hw — 자율주행 차량 PC

Livox MID-360 2대 + IMU로 **FAST-LIO 3D 측위**를 하고, 그 자세를 **평면으로 눌러서(z/roll/pitch 제거)**
**Nav2 2D 내비게이션**에 넘기는 차량 한 대분의 전체 설정.

이전에는 `ros2_ws` / `ws_livox` 두 워크스페이스에 흩어져 있었고 nav2는 62M짜리 소스 트리를
통째로 들고 있었다. 지금은 **워크스페이스 하나**로 합쳤다.

```
hw/
├── src/                        colcon 워크스페이스 (여기 하나뿐)
│   ├── description/            ★ 차량/센서 형상 URDF. TF의 유일한 출처
│   ├── livox_ros_driver2/      MID-360 x3 드라이버
│   ├── livox_merge/            front/rear 2대를 body 기준으로 병합
│   ├── fast_lio/               매핑 (맵 만들 때만)
│   ├── fast_lio_localization/  측위 + tf_2d 평면화
│   ├── pcd2pgm/                3D PCD -> 2D 점유맵
│   ├── motor_node/             CAN 구동
│   ├── teleop_twist_joy/       조이스틱 수동주행
│   └── hw_bringup/             ★ 전체 launch + Nav2 파라미터/BT/맵
├── third_party/Livox-SDK2/     colcon 대상 아님. sudo make install 필요
├── bag/                        rosbag 기록 위치 (git 제외)
└── docs/memo.md                예전 명령어 메모
```

Nav2 본체는 소스가 아니라 **apt(`/opt/ros/humble`)** 에서 온다. `hw_bringup` 은 실제로 손댄
파라미터·behavior tree·맵만 들고 있다.

---

## 데이터 흐름

```
MID-360 front ─┐
MID-360 rear  ─┴─ livox_merge ─ /livox_merge/merged_pointcloud ─┐
MID-360 top ───── /livox/imu_192_168_1_135 ────────────────────┤
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

TF 트리는 `map` 아래에서 두 갈래로 갈라진다.

```
map ─┬─ camera_init ── body ── livox_frame ─┬─ livox_top ── imu_link   (3D, FAST-LIO)
     │                                      ├─ livox_front
     │                                      └─ livox_rear
     └─ odom ── base_link                                             (2D, tf_2d.py)
```

`body` 아래 가지는 전부 `description` 패키지의 URDF 에서 나온다
(`robot_state_publisher`). 센서 위치를 바꿀 일이 있으면 `urdf/vehicle.urdf.xacro`
맨 위의 property 만 고치면 된다.

**Nav2는 `base_link` 쪽만 본다.** 이게 핵심이다 — `body` 를 보면 z와 roll/pitch가 그대로
따라 들어와서 코스트맵이 튄다.

---

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
ros2 launch hw_bringup bringup.launch.py
```

계층별로 나눠서:

```bash
ros2 launch hw_bringup sensors.launch.py                    # 라이다 2대 + IMU + 병합
ros2 launch fast_lio_localization localization.launch.py    # 측위 + 2D 평면화 + 맵서버
ros2 launch hw_bringup navigation_launch.py                 # Nav2
```

측위만 확인하고 싶으면 `ros2 launch hw_bringup bringup.launch.py navigation:=false`.

구동/수동주행은 `docs/memo.md` 참고.

## 맵 만들기

```bash
ros2 launch hw_bringup sensors.launch.py
ros2 launch fast_lio mapping.launch.py        # -> src/fast_lio/PCD/scans.pcd
ros2 run pcd2pgm pcd2pgm                      # 3D PCD -> 2D pgm
```

결과물을 `src/hw_bringup/maps/` 에 넣는다. 3D(`output.pcd`)와 2D(`scans_new.pgm`)는
**같은 주행에서 나온 짝이어야 한다** — 측위는 3D를, Nav2 코스트맵은 2D를 쓰기 때문에
둘이 어긋나면 로봇이 벽 속에 있다고 나온다.
