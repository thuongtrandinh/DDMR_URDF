import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get the path to the RViz config file
    pkg_path = get_package_share_directory('mobile_robot')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'view_robot.rviz')
    
    # Check if config file exists, if not use default
    if not os.path.exists(rviz_config_file):
        rviz_config_file = ''
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if rviz_config_file else [],
        parameters=[{
            'use_sim_time': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        rviz_node
    ])
