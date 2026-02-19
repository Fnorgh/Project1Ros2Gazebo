import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get package directory
    pkg_sim = get_package_share_directory('project_1')

    # Path to world file
    world = os.path.join(pkg_sim, 'worlds', 'enviromental.sdf')

    # Launch modern Gazebo (Harmonic / gz)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world, '-r'],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])
