from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('mobile_robot'),
            'xacro',
            'robot.xacro'
        ])
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
