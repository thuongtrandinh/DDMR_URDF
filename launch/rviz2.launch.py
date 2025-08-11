from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('mobile_robot'),
            'xacro',
            'robot.urdf.xacro'
        ])
    ])

    return LaunchDescription([
        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # joint_state_publisher_gui (điều khiển joint bằng slider GUI)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz (tùy chọn)
        # rviz2 with config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('mobile_robot'),
                    'rviz',
                    'rviz2.rviz'
                ])
            ],
            output='screen'
        )
    ])
