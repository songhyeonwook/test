import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_dir = os.path.join(get_package_share_directory('navigation'), 'urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(
                    Command(['xacro ',
                             os.path.join(urdf_dir, 'vehicle.urdf.xacro')]),
                    value_type=str),
            }]),
        # /docking(Bool) 에 따라 livox_frame->livox_top TF 를 주행/도킹모드로
        # 전환한다. 값은 mount.xacro 의 top_xyz/top_rpy, top_dock_* 를 읽는다.
        Node(
            package='navigation',
            executable='top_tf_switcher.py',
            name='top_tf_switcher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
    ])
