# nav2 (커스터마이즈 파일만)

nav2 본체는 apt(`/opt/ros/humble`)에 이미 설치되어 있다. `ros2_ws/src/navigation2`
소스 트리(62M)는 한 번도 빌드된 적이 없어서 런타임에 아무 영향이 없었고, 그 안에서
직접 수정하거나 새로 만든 파일만 여기로 추린 것이다.

| 경로 | 내용 |
|---|---|
| `params/nav2_params.yaml` | `robot_base_frame: body`, Jetson용 BT 틱/타임아웃 완화, 컨트롤러·goal tolerance 튜닝 |
| `launch/navigation_launch.py` | 아래 커스텀 BT를 기본값으로 물리도록 수정 |
| `behavior_trees/navigate_{to_pose,through_poses}_vehicle.xml` | 차량형(Ackermann) 전용 BT — 업스트림에 없음 |
| `maps/scans*.{pgm,yaml}` | FAST-LIO 맵을 pcd2pgm으로 변환한 실측 맵 |

원본 소스 트리는 커밋 `bddce1e` 에 통째로 남아 있다.

## 실행

```bash
ros2 launch ~/hw/nav2/launch/navigation_launch.py \
  use_sim_time:=false \
  params_file:=$HOME/hw/nav2/params/nav2_params.yaml
```

`navigation_launch.py` 는 `nav2_bringup` 패키지 share 경로가 아니라 **자기 파일 위치 기준**
으로 BT를 찾도록 고쳐놨다. 이 디렉토리를 통째로 옮기면 그대로 따라간다.

맵은 map_server에 따로 넘긴다:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$HOME/hw/nav2/maps/scans_new.yaml
```
