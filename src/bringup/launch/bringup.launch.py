"""이 PC 한 대의 전체 스택.

  sensors      : 라이다 3대 + 내장 IMU + 병합
  localization : FAST-LIO 3D 측위 -> tf_2d 평면화 -> map_server(2D 점유맵)
  navigation   : Nav2

인자:
  sensors:=false        센서를 이미 따로 띄웠을 때
  localization:=false   측위를 이미 따로 띄웠을 때
  navigation:=false     측위만 확인하고 싶을 때
  motor:=false          하부 모터노드(CAN)를 따로 띄우거나 없는 PC 에서
  lift:=false           리프트 제어보드(RS232 /dev/ttyTHS1)가 없는 PC 에서
  app:=true             rbio TransferRobot 앱과 함께 돌릴 때 (기본은 env HW_APP_MODE, 없으면 false)
                        - Nav2 는 /cmd_vel_nav 까지만 내고 앱이 /cmd_vel 로 중계
                        - motor_node 는 센터링 뒤 자동으로 애커만 진입, 도킹 속도 0.04/0.05
  rviz:=false           rviz 없이
  map:=<이름>           navigation/map/<이름>.yaml(2D) + <이름>.pcd(3D) 를 쓴다
ros2 launch bringup bringup.launch.py map:=<이름>   

"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    loc_dir = get_package_share_directory('fast_lio_localization')
    nav_dir = get_package_share_directory('navigation')

    sensors = LaunchConfiguration('sensors')
    localization = LaunchConfiguration('localization')
    navigation = LaunchConfiguration('navigation')
    rviz = LaunchConfiguration('rviz')
    motor = LaunchConfiguration('motor')
    lift = LaunchConfiguration('lift')
    app = LaunchConfiguration('app')
    goal_recorder = LaunchConfiguration('goal_recorder')
    goal_log_dir = LaunchConfiguration('goal_log_dir')
    collision_monitor = LaunchConfiguration('collision_monitor')
    dynamic_tracker = LaunchConfiguration('dynamic_tracker')
    app_true = ["'", app, "' == 'true'"]
    collision_cmd_vel_out = PythonExpression(
        ["'cmd_vel_nav' if "] + app_true + [" else 'cmd_vel'"])
    emergency_cmd_vel_out = PythonExpression(
        ["'cmd_vel_dynamic_monitor_in' if '", dynamic_tracker,
         "' == 'true' else ('cmd_vel_nav' if "] + app_true
        + [" else 'cmd_vel')"])
    map_name = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('motor', default_value='true'),
        DeclareLaunchArgument('lift', default_value='true'),
        DeclareLaunchArgument('app', default_value=EnvironmentVariable('HW_APP_MODE', default_value='false')),
        DeclareLaunchArgument(
            'goal_recorder', default_value='true',
            description='Nav2 goal result and 70 mm error CSV recorder'),
        DeclareLaunchArgument(
            'goal_log_dir', default_value='~/nav_goal_results',
            description='Directory for field-platform goal result CSV'),
        DeclareLaunchArgument(
            'collision_monitor', default_value='true',
            description='Directional raw collision gate'),
        DeclareLaunchArgument(
            'dynamic_tracker', default_value='true',
            description='360-degree dynamic-obstacle stop and resume tracker'),

        Node(
            package='navigation',
            executable='goal_error_recorder_HY.py',
            name='goal_error_recorder',
            output='both',
            condition=IfCondition(goal_recorder),
            parameters=[{
                'output_directory': goal_log_dir,
                'position_limit_mm': 70.0,
                # 출장 FAST-LIO는 실제 bag에서 localization 수신 지연이
                # 1.6초를 넘는 경우가 있다. 성공 이후 stamp 조건은 recorder가
                # 별도로 강제하므로 처리 지연만 2.5초까지 허용한다.
                'max_localization_age_ms': 2500.0,
                'fresh_sample_wait_ms': 2500.0,
                # RViz/ros2 topic pub의 QoS와 무관하게 실제 명령 Goal을 받고,
                # grid에 스냅된 plan 끝점으로 오차를 잘못 판정하지 않는다.
                'goal_best_effort_qos': True,
                'allow_plan_fallback_measurement': False,
                # KTL 출장 시험은 아래 고정 좌표를 반복한다. /goal_pose 단발
                # 메시지를 DDS가 놓쳐도 인접한 plan을 실제 기준 Goal로 복원한다.
                'reference_goal_enabled': True,
                'reference_goal_x': -7.355742617115387,
                'reference_goal_y': 2.1501534559125135,
                'reference_goal_yaw_deg': 150.20799034716626,
                'reference_goal_match_radius': 0.20,
            }]),

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
        # 리프트 제어보드: lift/vertical, lift/horizontal (Int32 0/1/2) -> RS232 프레임
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('lift_node'),
                             'launch', 'lift.launch.py')),
            condition=IfCondition(lift)),
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
                'collision_monitor': collision_monitor,
            }.items()),

        Node(
            package='navigation',
            executable='dynamic_obstacle_tracker.py',
            name='dynamic_obstacle_tracker_HY',
            output='both',
            condition=IfCondition(dynamic_tracker),
            parameters=[os.path.join(nav_dir, 'params', 'nav2_params.yaml')]),

        Node(
            package='navigation',
            executable='directional_collision_gate_HY.py',
            name='directional_collision_gate_HY',
            output='both',
            condition=IfCondition(collision_monitor),
            parameters=[os.path.join(nav_dir, 'params', 'nav2_params.yaml')],
            remappings=[
                ('/cmd_vel_collision_monitor_out', emergency_cmd_vel_out),
            ]),

        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='dynamic_collision_monitor',
            output='both',
            condition=IfCondition(dynamic_tracker),
            parameters=[os.path.join(nav_dir, 'params', 'nav2_params.yaml')],
            remappings=[
                ('cmd_vel_dynamic_monitor_out', collision_cmd_vel_out),
            ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_dynamic_monitor',
            output='screen',
            condition=IfCondition(dynamic_tracker),
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['dynamic_collision_monitor'],
            }]),
    ])
