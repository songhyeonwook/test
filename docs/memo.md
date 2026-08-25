# 자주 쓰는 명령

전부 `conda deactivate` 후 `source ~/hw/install/setup.bash` 를 먼저 한다.

## 구동

```bash
alias can='~/hw/src/motor_node/script/can.sh'
alias motor='ros2 launch motor_node bringup.launch.py'
alias init='ros2 run motor_node drive_set.py'
alias monitor='ros2 run motor_node state_monitor.py'
```

## 센서 / 측위 / 내비

```bash
ros2 launch bringup bringup.launch.py                 # 전부 한 번에
ros2 launch bringup sensors.launch.py                 # 라이다 3대 + 병합 + TF
ros2 launch fast_lio_localization localization.launch.py
ros2 launch bringup navigation_launch.py              # Nav2
```

측위만 확인: `ros2 launch bringup bringup.launch.py navigation:=false`

## 맵 만들기

```bash
ros2 launch bringup sensors.launch.py
ros2 launch fast_lio mapping.launch.py     # -> src/fast_lio/PCD/scans.pcd
ros2 run pcd2pgm pcd2pgm                   # 3D PCD -> 2D pgm
```
결과물은 `src/bringup/maps/` 로. 3D(output.pcd)와 2D(scans_new.pgm)는 같은
주행에서 나온 짝이어야 한다.

## bag

```bash
cd ~/hw/bag && ros2 bag record -a
ros2 launch bringup bag_localization.launch.py rate:=0.3   # 재생 측위 테스트
```
재생 중 RViz 의 "2D Pose Estimate" 로 초기 자세를 찍어야 측위가 붙는다.

## 확인

```bash
python3 tools/check_frames.py                       # URDF <-> 설정 정합
rviz2 -d src/description/rviz/frames_check.rviz     # 모델과 클라우드 겹쳐보기
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
```

## 아직 미정

- Nav2 코스트맵이 static_layer 만 쓴다. 라이브 장애물 레이어를 붙이려면
  `sudo apt install ros-humble-pointcloud-to-laserscan` 후
  `/livox_merge/merged_pointcloud_sliced` 를 `/scan` 으로 변환해서 넣는다.
- `mount.xacro` 의 `top_xyz` z 값(0.8465)이 실측 전 추정치다.
