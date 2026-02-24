import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_project_1 = get_package_share_directory('project_1')
    pkg_tb4_gz = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_tb4_gz_gui = get_package_share_directory('turtlebot4_gz_gui_plugins')
    pkg_irobot_gz = get_package_share_directory('irobot_create_gz_bringup')
    pkg_irobot_gz_pl = get_package_share_directory('irobot_create_gz_plugins')
    pkg_tb4_desc = get_package_share_directory('turtlebot4_description')
    pkg_irobot_desc = get_package_share_directory('irobot_create_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_project_1, 'worlds', 'enviromental.sdf')
    gui_config_file = os.path.join(pkg_tb4_gz, 'gui', 'standard', 'gui.config')

    # ── Launch args ────────────────────────────────────────────────────────────
    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    # ── Environment variables (set before Gazebo starts) ──────────────────────
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_project_1, 'worlds'),
            os.path.join(pkg_project_1, 'models'),
            os.path.join(pkg_tb4_gz, 'worlds'),
            os.path.join(pkg_irobot_gz, 'worlds'),
            str(Path(pkg_tb4_desc).parent.resolve()),
            str(Path(pkg_irobot_desc).parent.resolve()),
        ])
    )

    gz_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=':'.join([
            os.path.join(pkg_tb4_gz_gui, 'lib'),
            os.path.join(pkg_irobot_gz_pl, 'lib'),
        ])
    )

    gz_sim_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/jazzy/lib'
    )

    # ── Gazebo (ROS-managed launch; more reliable for ros2_control) ───────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            # NOTE: gz_args is a single string; include GUI config here.
            'gz_args': f'-r -v 4 --gui-config "{gui_config_file}" "{world_file}"',
        }.items(),
    )

    # ── Spawn TurtleBot4 (bridges + controllers) ──────────────────────────────
    spawn_tb4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_gz, 'launch', 'turtlebot4_spawn.launch.py'])
        ),
        launch_arguments=[
            ('x', LaunchConfiguration('x_pose')),
            ('y', LaunchConfiguration('y_pose')),
            ('yaw', LaunchConfiguration('yaw')),
        ]
    )

    # Give Gazebo a moment to fully start before TB4 spawn/controllers
    delayed_spawn_tb4 = TimerAction(period=5.0, actions=[spawn_tb4])

    # (Optional) bumper stop node (only works if /bumper_contact actually publishes contacts)
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
        declare_x_pose,
        declare_y_pose,
        declare_yaw,
        gz_resource_path,
        gz_gui_plugin_path,
        gz_sim_plugin_path,
        gazebo,
        delayed_spawn_tb4,
        bumper_stop,
    ])