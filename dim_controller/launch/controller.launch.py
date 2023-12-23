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


    package_path = get_package_share_path('dim_controller')
    controller_params_file_path = str(package_path / 'config/controller_params.yaml')

    controller_params_file_arg = DeclareLaunchArgument('controller_config_file',
                                           default_value=controller_params_file_path)
    launch_description.add_action(controller_params_file_arg)

    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        Node(executable='position_controller.py',
             package='dim_controller',
                parameters=[LaunchConfiguration('controller_config_file')]),
            
        # add more here
    ])
    launch_description.add_action(group)
    return launch_description
