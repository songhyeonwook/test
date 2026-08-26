from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    teleop_launch_path = os.path.join(
        get_package_share_directory('teleop_twist_joy'),
        'launch',
        'teleop-launch.py'
    )

    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('motor_node'), 'launch', 'motor.launch.py')))

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_path)
    )

    return LaunchDescription([
        motor_launch,
        teleop_launch
    ])
