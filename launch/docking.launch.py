from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('waver_docking')
    
    # Get the paths to the parameter and rviz config files
    params_file = os.path.join(pkg_dir, 'params', 'docking_params.yaml')
    
    return LaunchDescription([
        # Launch the docking node
        Node(
            package='waver_docking',
            executable='docking_node',
            name='docking_node',
            output='screen',
            parameters=[params_file]
        ),
        
        # Launch the interactive graph node
        Node(
            package='waver_docking',
            executable='interactive_points_node.py',
            name='interactive_points_node',
            output='screen',
            parameters=[params_file]
        )
    ])
