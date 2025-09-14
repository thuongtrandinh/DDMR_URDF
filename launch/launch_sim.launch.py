
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_name = 'mobile_robot'
    use_sim_time_str = context.launch_configurations['use_sim_time']
    use_sim_time = use_sim_time_str.lower() == 'true'  # Convert string to boolean
    init_height = context.launch_configurations['height']
    
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'mobile_robot.urdf.xacro')
    
    # 🔥 PROPER XACRO RENDERING: Convert XACRO to URDF XML
    robot_description = xacro.process_file(xacro_file, mappings={'sim_mode': 'true'}).toxml()
    
    # 💡 Debug: Print rendered URDF size for verification
    print(f"✅ URDF successfully rendered! Size: {len(robot_description)} characters")

    # RViz config file
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'rviz2.rviz')

    # RViz2 node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Robot State Publisher with enhanced QoS for RViz2 compatibility
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
            # 🔧 CRITICAL: QoS override for RViz2 compatibility
            'qos_overrides./robot_description.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Gazebo spawn entity with timing
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-z', init_height],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Controllers with proper timing
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Bridge - CRITICAL: Clock bridge MUST be first for time sync
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/diff_cont/cmd_vel_unstamped@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Gazebo launch - IMPORTANT: Force simulation time and sync
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments=[
            ('gz_args', ['-r -v 4 empty.sdf']),
            ('use_sim_time', 'true'),
            ('sync', 'true'),  # Force time synchronization
        ]
    )

    return [
        # 🔥 CRITICAL ORDER: Bridge first for time sync, then Gazebo
        bridge,
        gazebo_launch,
        robot_state_publisher,
        rviz,
        gz_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[diff_drive_controller],
            )
        ),
    ]


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    height_arg = DeclareLaunchArgument(
        'height',
        default_value='0.2',
        description='Initial spawn height in simulation'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )

    return LaunchDescription([
        use_sim_time_arg,
        height_arg,
        gui_arg,
        OpaqueFunction(function=launch_setup),
    ])
