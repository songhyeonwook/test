"""녹화된 bag 으로 측위를 테스트한다.

  ros2 launch bringup bag_localization.launch.py bag:=/home/shw/hw/bag/bag/rosbag2_ackermann

bag 이 실장비와 다른 점:
  - 라이다가 PointCloud2 다(xfer_format=0). 실장비도 지금은 같은 형식이라 병합
    노드 설정은 clouds 만 다르다(기존 bag 은 front/rear 만 녹화돼 있어 2).
    top 까지 들어 있는 bag 이면 clouds:=3 으로 띄운다.
  - bag 안에 /tf, /tf_static 이 들어 있다. 녹화 당시의 static TF 와 휠 오도메트리
    odom->base_link 인데, 지금은 description 의 URDF 와 tf_2d.py 가 같은 프레임을
    내보내므로 부모가 둘이 되어 TF 가 깨진다. humble 의 ros2 bag play 에는
    제외 옵션이 없어서 --topics 로 필요한 것만 골라 재생한다.
  - 시간은 bag 시계를 쓴다(--clock + use_sim_time:=true).

RViz 에서 "2D Pose Estimate" 로 초기 자세를 찍어주면 global_localization 이
ICP 로 맞춘다. 지도와 겹치는 지점을 대충 찍어도 된다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# sensors.launch.py 의 CONCAT_REMAPPINGS 와 같은 내용. launch 파일끼리는 import
# 가 안 되므로 여기에 한 벌 둔다. 한쪽을 고치면 다른 쪽도 고칠 것.
CONCAT_REMAPPINGS = [
    ('cloud_in1', '/livox/lidar_192_168_2_102'),          # livox_front
    ('cloud_in2', '/livox/lidar_192_168_3_144'),          # livox_rear
    ('cloud_in3', '/livox/lidar_192_168_1_135'),          # livox_top
    ('cloud_out', '/livox_merge/merged_pointcloud'),
    ('cloud_out_sliced', '/livox_merge/merged_pointcloud_sliced'),
]


def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    nav_dir = get_package_share_directory('navigation')
    loc_dir = get_package_share_directory('fast_lio_localization')

    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')
    rviz = LaunchConfiguration('rviz')

    merge_params = os.path.join(bringup_dir, 'params', 'livox_merge_bag.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag',
            default_value=os.path.expanduser('~/hw/bag/bag/rosbag2_ackermann'),
            description='재생할 rosbag 디렉토리'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'clouds', default_value='2',
            description='병합할 라이다 수. bag 에 top 까지 있으면 3'),
        DeclareLaunchArgument('map', default_value='test'),

        # 차량/센서 TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_dir, 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),

        # 라이다 병합 (TF 로 base_link 기준 합침)
        Node(
            package='pointcloud_concatenate_ros2',
            executable='pointcloud_concat_node',
            name='livox_merge',
            output='screen',
            parameters=[merge_params,
                        {'use_sim_time': True,
                         'clouds': ParameterValue(LaunchConfiguration('clouds'),
                                                  value_type=int)}],
            remappings=CONCAT_REMAPPINGS),

        # FAST-LIO 측위 + tf_2d 평면화 + 2D 맵서버
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_dir, 'launch', 'localization.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'rviz': rviz,
                              'map': LaunchConfiguration('map')}.items()),

        # bag 재생. tf 계열은 지금 노드들이 다시 만들므로 재생하지 않는다.
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate,
                 '--topics',
                 '/livox/lidar_192_168_2_102',   # front -> merge
                 '/livox/lidar_192_168_3_144',   # rear  -> merge
                 '/livox/lidar_192_168_1_135',   # top   -> merge (clouds:=3 일 때만 쓰인다.
                                                 #          bag 에 없으면 그냥 무시된다)
                 '/livox/imu_192_168_1_135',     # top 내장 IMU -> FAST-LIO
                 '/odom'],                       # 휠 odom -> nav2 속도 피드백
            output='screen'),
    ])
