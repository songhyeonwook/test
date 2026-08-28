# 상류 저장소의 데모 launch (TI mmWave 레이더 2대). 이 로봇에서는 쓰지 않는다.
# 실제 구성은 bringup 쪽에 있다:
#   src/bringup/params/livox_merge.yaml        실장비
#   src/bringup/params/livox_merge_bag.yaml    bag 재생
#   src/bringup/launch/sensors.launch.py       노드 기동
#
# 파라미터 이름에서 앞의 '/' 를 뗐다(YAML 파라미터 파일에서 쓰기 위해).
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'map', '--child-frame-id', 'ti_mmwaver_0'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '-0.05', '--y', '0.5', '--z', '0',
                '--yaw', '1.571', '--pitch', '0', '--roll', '0',
                '--frame-id', 'ti_mmwaver_0', '--child-frame-id', 'ti_mmwaver_1'
            ]
        ),
        Node(
            package="pointcloud_concatenate_ros2",
            executable="pointcloud_concat_node",
            parameters=[{
                "target_frame": "map",
                "clouds": 2,
                "hz": 10.0
            }],
            remappings=[("cloud_in1","/ti_radar_0/ti_mmwave_0"),
                        ("cloud_in2","/ti_radar_1/ti_mmwave_1"),
                        ("cloud_out","fusion")]
        )
    ])
