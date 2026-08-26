"""이 PC 한 대의 전체 스택.

  sensors      : 라이다 2대 + IMU + 병합
  localization : FAST-LIO 3D 측위 -> tf_2d 평면화 -> map_server(2D 점유맵)
  navigation   : Nav2

인자:
  sensors:=false        센서를 이미 따로 띄웠을 때
  localization:=false   측위를 이미 따로 띄웠을 때
  navigation:=false     측위만 확인하고 싶을 때
  motor:=false          하부 모터노드(CAN)를 따로 띄우거나 없는 PC 에서
  app:=true             rbio TransferRobot 앱과 함께 돌릴 때 (기본은 env HW_APP_MODE, 없으면 false)
                        - Nav2 는 /cmd_vel_nav 까지만 내고 앱이 /cmd_vel 로 중계
                        - motor_node 는 센터링 뒤 자동으로 애커만 진입, 도킹 속도 0.04/0.05
  rviz:=false           rviz 없이
  map:=<이름>           navigation/map/<이름>.yaml(2D) + <이름>.pcd(3D) 를 쓴다
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    loc_dir = get_package_share_directory('fast_lio_localization')
    nav_dir = get_package_share_directory('navigation')

    sensors = LaunchConfiguration('sensors')
    localization = LaunchConfiguration('localization')
    navigation = LaunchConfiguration('navigation')
    rviz = LaunchConfiguration('rviz')
    motor = LaunchConfiguration('motor')
    app = LaunchConfiguration('app')
    app_true = ["'", app, "' == 'true'"]
    map_name = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('motor', default_value='true'),
        DeclareLaunchArgument('app', default_value=EnvironmentVariable('HW_APP_MODE', default_value='false')),

        # 하부 모터노드: /cmd_vel -> CAN, /odom(4WS 휠 오도메트리) -> Nav2 속도 피드백
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('motor_node'),
                             'launch', 'motor.launch.py')),
            condition=IfCondition(motor),
            launch_arguments={
                'startup_mode': PythonExpression(["'1' if "] + app_true + [" else '0'"]),
                'diff_max_linear_vel': PythonExpression(["'0.04' if "] + app_true + [" else '0.0'"]),
                'diff_max_angular_vel': PythonExpression(["'0.05' if "] + app_true + [" else '0.0'"]),
            }.items()),
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
                'app': app,
            }.items()),
    ])
