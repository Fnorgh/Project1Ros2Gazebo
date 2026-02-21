import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_sim = get_package_share_directory('project_1')
    pkg_tb4_gz = get_package_share_directory('turtlebot4_gz_bringup')

    world = os.path.join(pkg_sim, 'worlds', 'enviromental.sdf')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw    = LaunchConfiguration('yaw',    default='0.0')

    declare_x   = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y   = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw',    default_value='0.0')

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world, '-r'],
        output='screen'
    )

    spawn_tb4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_gz, 'launch', 'turtlebot4_spawn.launch.py')
        ),
        launch_arguments={
            'x': x_pose,
            'y': y_pose,
            'yaw': yaw,
        }.items(),
    )

    return LaunchDescription([
        declare_x,
        declare_y,
        declare_yaw,
        gazebo,
        spawn_tb4,
    ])