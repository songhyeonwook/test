from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 리프트 제어보드 RS232. 앱(run_transfer_robot.sh) 기본값과 같다.
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        Node(
            package='lift_node',
            executable='lift_node',
            name='lift_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                # 보드는 마지막 지령을 래치한다. 1/2 지령이 이 시간 안에 다시 안 오면
                # 정지를 보낸다. 0 = 래치 그대로 (앱 방식: 누를 때 1/2, 뗄 때 0 한 번).
                'cmd_timeout_s': 0.5,
                # 연결 직후 0x40(로봇암 오류 조회) 을 보내 왕복 응답으로 링크를 점검한다.
                # 응답이 없어도 지령은 보낼 수 있다 (상태 NO_RESPONSE, 진단 WARN).
                'probe_on_connect': True,
                'probe_timeout_s': 2.5,
                'reconnect_period_s': 2.0,
                # lift_node/move 액션에서 duration 을 안 주면 쓰는 방향별 전 구간 시간.
                # 벤치 측정값 (상승 지령 → 최대점 정지까지 약 4.2 s). 수평은 아직 미측정.
                'vertical_up_duration_s': 4.2,
                'vertical_down_duration_s': 4.2,
                'horizontal_extend_duration_s': 3.0,
                'horizontal_retract_duration_s': 3.0,
                'max_move_duration_s': 30.0,
            }]
        ),
    ])
