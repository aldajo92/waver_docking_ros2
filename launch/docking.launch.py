from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('waver_docking'),
        'params',
        'docking_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='waver_docking',
            executable='docking_node',
            name='docking_node',
            output='screen',
            parameters=[params_file]
        )
    ])