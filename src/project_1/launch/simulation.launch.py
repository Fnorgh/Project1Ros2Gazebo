import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_project_1    = get_package_share_directory('project_1')
    pkg_tb4_gz       = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_tb4_gz_gui   = get_package_share_directory('turtlebot4_gz_gui_plugins')
    pkg_irobot_gz    = get_package_share_directory('irobot_create_gz_bringup')
    pkg_irobot_gz_pl = get_package_share_directory('irobot_create_gz_plugins')
    pkg_tb4_desc     = get_package_share_directory('turtlebot4_description')
    pkg_irobot_desc  = get_package_share_directory('irobot_create_description')

    world_file      = os.path.join(pkg_project_1, 'worlds', 'enviromental.sdf')
    gui_config_file = os.path.join(pkg_tb4_gz, 'gui', 'standard', 'gui.config')

    # ── Launch args ────────────────────────────────────────────────────────
    declare_x_pose    = DeclareLaunchArgument('x_pose',    default_value='0.0')
    declare_y_pose    = DeclareLaunchArgument('y_pose',    default_value='0.0')
    declare_yaw       = DeclareLaunchArgument('yaw',       default_value='0.0')
    # FIX #1: declare namespace so turtlebot4_spawn.launch.py can find it
    declare_namespace = DeclareLaunchArgument('namespace', default_value='')

    # ── Global sim time for all nodes ──────────────────────────────────────
    set_sim_time = SetParameter(name='use_sim_time', value=True)

    # ── Environment variables ───────────────────────────────────────────────
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_project_1, 'worlds'),
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

    # ── Gazebo ──────────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r', '-v', '4', '--gui-config', gui_config_file],
        output='screen'
    )

    # ── Clock bridge ────────────────────────────────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ── Spawn TurtleBot4 ────────────────────────────────────────────────────
    # FIX #1: pass namespace='' so the LaunchConfiguration exists inside spawn
    # FIX #2: increase delay to 8s so controller_manager is up before spawner runs
    spawn_tb4 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_tb4_gz, 'launch', 'turtlebot4_spawn.launch.py'])
                ),
                launch_arguments=[
                    ('x',         LaunchConfiguration('x_pose')),
                    ('y',         LaunchConfiguration('y_pose')),
                    ('yaw',       LaunchConfiguration('yaw')),
                    ('namespace', LaunchConfiguration('namespace')),  # FIX #1
                ]
            )
        ]
    )

    # ── bumper_stop ─────────────────────────────────────────────────────────
    bumper_stop = TimerAction(
        period=15.0,   # well after robot is fully spawned + controllers active
        actions=[
            Node(
                package='project_1',
                executable='bumper_stop',
                name='bumper_stop',
                parameters=[
                    {'contacts_topic': '/bumper_contact'},
                    {'stop_topic':     '/stop_request'},
                    {'publish_hz':     50.0},
                    {'stop_hold_sec':  1.5},
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        set_sim_time,        # MUST be first
        declare_x_pose,
        declare_y_pose,
        declare_yaw,
        declare_namespace,   # FIX #1: must be declared before spawn uses it
        gz_resource_path,
        gz_gui_plugin_path,
        gz_sim_plugin_path,
        gazebo,
        clock_bridge,
        spawn_tb4,
        bumper_stop,
    ])