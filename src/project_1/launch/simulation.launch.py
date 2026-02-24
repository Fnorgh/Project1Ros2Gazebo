import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_tb4_gz = get_package_share_directory('turtlebot4_gz_bringup')

    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    spawn_tb4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_gz, 'launch', 'turtlebot4_spawn.launch.py')
        ),
        launch_arguments={
            'gazebo': 'ignition',
            'world': 'enviromental',   
            'x': LaunchConfiguration('x_pose'),
            'y': LaunchConfiguration('y_pose'),
            'yaw': LaunchConfiguration('yaw'),
            'use_sim_time': 'true',
            'rviz': 'false',
            'nav2': 'false',
            'slam': 'false',
            'localization': 'false',
        }.items(),
    )

    bumper_stop = Node(
        package='project_1',
        executable='bumper_stop',
        name='bumper_stop',
        parameters=[
            {'contacts_topic': '/bumper_contact'},
            {'cmd_vel_topic': '/diffdrive_controller/cmd_vel'},
            {'publish_hz': 50.0},
            {'stop_hold_sec': 1.5},
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_x_pose, declare_y_pose, declare_yaw,
        spawn_tb4,
        bumper_stop,
    ])