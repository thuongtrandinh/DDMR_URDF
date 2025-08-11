
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription , DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

"""
gazebo_launch = IncludeLaunchDescription(
    PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ]),
    launch_arguments={
        'gz_args': ['-r empty.sdf'],
        'gui': LaunchConfiguration('gui'),
        'on_exit_shutdown': 'true'
    }.items()
)
"""


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='mobile_robot' #<--- CHANGE ME
    gui_arg = DeclareLaunchArgument(
            name='gui',
            default_value='true',
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    gazebo_launch = IncludeLaunchDescription(
    PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ]),
    launch_arguments={
        # KHÔNG để --force-version ở đây nữa
        'gz_args': '-r empty.sdf',
        # Ép dùng Harmonic qua tham số riêng của launch file
        'gz_version': '8',
        'gui': LaunchConfiguration('gui'),
        'on_exit_shutdown': 'true'
    }.items()
    )
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                   '-z', '0.2'],
                        output='screen')



    # Launch them all!
    return LaunchDescription([
        gui_arg,
        rsp,
        gazebo_launch,
        spawn_entity,
    ])
