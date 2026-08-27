from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 시동 센터링 뒤 자동 진입 모드. 0 = /mode 대기(단독 테스트), 1 = 애커만(rbio 앱 모드)
        DeclareLaunchArgument('startup_mode', default_value='0'),
        # 디프(도킹) 전용 속도 상한. 0 = 전역 상한. rbio 도킹은 0.04 / 0.05 를 썼다.
        DeclareLaunchArgument('diff_max_linear_vel', default_value='0.0'),
        DeclareLaunchArgument('diff_max_angular_vel', default_value='0.0'),
        Node(
            package='motor_node',
            executable='motor_node',
            name='motor_node',
            output='screen',
            parameters=[{
                # 조향 Profile Position (rbio 기본값)
                'steering_profile_velocity': 50000,
                'steering_profile_acceleration': 50000,
                'steering_profile_deceleration': 80000,
                'max_steering_angle_deg': 45.0,
                # 주행 프로파일. 0x6083(가속) / 0x6084(감속) 을 따로 준다.
                # 감속을 크게 잡으면 정지가 빨라지고, 작게 잡으면 부드럽게 선다.
                'drive_profile_acceleration': 50000,
                'drive_profile_deceleration': 80000,
                # cmd_vel 상한 / 타임아웃
                'max_linear_vel': 0.5, #0.2
                'max_angular_vel': 0.5, #0.3
                'cmd_vel_timeout_s': 0.5,
                # 디프 모드 (/mode 2): 전후륜을 이 각도로 돌려 고정. 0 이면 공통값 사용.
                # linear.x(또는 y) = 횡이동(+ 좌), angular.z = 제자리 회전
                'diff_steer_deg': 90.0,
                'diff_steer_front_deg': 0.0,
                'diff_steer_rear_deg': 0.0,
                'diff_max_linear_vel': LaunchConfiguration('diff_max_linear_vel'),
                'diff_max_angular_vel': LaunchConfiguration('diff_max_angular_vel'),
                'startup_mode': LaunchConfiguration('startup_mode'),
                'diff_linear_x_is_lateral': True,
                # JOY 모드 (/mode 3): 조이스틱 수동. 조향축을 속도제어(PV)로 바꿔 쓴다.
                # angular.z 를 max_angular_vel 로 정규화해 풀 스케일에서 이 조향속도가 나온다.
                'joy_steer_rate_deg_s': 20.0,
                'joy_max_linear_vel': 0.0,  # 0 = 전역 max_linear_vel
                'joy_stick_deadzone': 0.05,
                # 4WS 휠 오도메트리. odom->base_link TF 는 fast_lio 의 tf_2d.py 가
                # 내므로 여기서는 /odom 토픽만 낸다. 단독 테스트 시 True 로.
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'publish_odom_tf': False,
                'odom_wheel_deadband_mps': 0.005,
            }]
        ),
    ])
