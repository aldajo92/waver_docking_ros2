from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('waver_docking')
    
    # Get the paths to the parameter and rviz config files
    params_file = os.path.join(pkg_dir, 'params', 'points_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'show_points.rviz')
    
    return LaunchDescription([
        # Launch the interactive graph node
        Node(
            package='waver_docking',
            executable='interactive_points_node.py',
            name='interactive_points_node',
            parameters=[params_file],
            output='screen'
        ),
        
        # Launch RViz2 with our configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
