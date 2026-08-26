"""이 PC 한 대의 전체 스택.

  sensors      : 라이다 2대 + IMU + 병합
  localization : FAST-LIO 3D 측위 -> tf_2d 평면화 -> map_server(2D 점유맵)
  navigation   : Nav2

인자:
  sensors:=false        센서를 이미 따로 띄웠을 때
  localization:=false   측위를 이미 따로 띄웠을 때
  navigation:=false     측위만 확인하고 싶을 때
  rviz:=false           rviz 없이
  map:=<이름>           navigation/map/<이름>.yaml(2D) + <이름>.pcd(3D) 를 쓴다
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    loc_dir = get_package_share_directory('fast_lio_localization')
    nav_dir = get_package_share_directory('navigation')

    sensors = LaunchConfiguration('sensors')
    localization = LaunchConfiguration('localization')
    navigation = LaunchConfiguration('navigation')
    rviz = LaunchConfiguration('rviz')
    map_name = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value='test'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'sensors.launch.py')),
            condition=IfCondition(sensors)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_dir, 'launch', 'localization.launch.py')),
            condition=IfCondition(localization),
            launch_arguments={'rviz': rviz, 'use_sim_time': 'false',
                              'map': map_name}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_dir, 'launch', 'navigation_launch.py')),
            condition=IfCondition(navigation),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(nav_dir, 'params', 'nav2_params.yaml'),
            }.items()),
    ])
