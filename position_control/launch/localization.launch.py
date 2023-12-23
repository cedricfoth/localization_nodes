from launch_ros.actions import Node, PushRosNamespace

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    package_path = get_package_share_path('position_control')
    params_file_path = str(package_path / 'config/position_estimation_params.yaml')

    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        Node(executable='position_estimation.py',
             package='position_control',
             parameters=[
                 LaunchConfiguration('params_file_path',
                                     default=params_file_path)
             ]),
        Node(executable='ranges_debugger.py', package='position_control')
        # add more here
    ])
    launch_description.add_action(group)
    return launch_description
