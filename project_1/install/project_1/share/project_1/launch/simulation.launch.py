"""Phase 1 launch file — starts Gazebo with the environmental world,
spawns TurtleBot4, bridges ROS↔Gazebo topics, and starts SLAM Toolbox."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_1 = get_package_share_directory('project_1')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_tb4_desc = get_package_share_directory('nav2_minimal_tb4_description')
    pkg_tb4_nav = get_package_share_directory('turtlebot4_navigation')

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_x = DeclareLaunchArgument(
        'x_pose', default_value='-0.762',
        description='Initial X position of the robot (room centre)')
    declare_y = DeclareLaunchArgument(
        'y_pose', default_value='0.762',
        description='Initial Y position of the robot (room centre)')
    declare_yaw = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='Initial yaw of the robot')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Paths ─────────────────────────────────────────────────────────────────
    world_file = os.path.join(pkg_project_1, 'worlds', 'enviromental.sdf')
    robot_xacro = os.path.join(
        pkg_tb4_desc, 'urdf', 'standard', 'turtlebot4.urdf.xacro')
    bridge_config = os.path.join(pkg_project_1, 'config', 'tb4_bridge.yaml')

    # ── GZ_SIM_RESOURCE_PATH: local models + tb4 description ─────────────────
    # Lets Gazebo find both the local turtlebot4 model and the stock one.
    set_resource_path_local = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_project_1, 'models'))
    set_resource_path_tb4 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(pkg_tb4_desc).parent.resolve()))

    # ── Gazebo ────────────────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items())

    # ── Robot state publisher (TF tree) ──────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', robot_xacro]),
            'use_sim_time': use_sim_time,
        }])

    # ── Spawn TurtleBot4 in Gazebo (delayed so the server is ready) ──────────
    spawn_robot = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_turtlebot4',
            output='screen',
            arguments=[
                '-entity', 'turtlebot4',
                '-topic', 'robot_description',
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01',
                '-Y', yaw,
            ])])

    # ── ROS ↔ Gazebo bridge ───────────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }])

    # ── SLAM Toolbox (builds the occupancy grid from /scan + /odom) ───────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_nav, 'launch', 'slam.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'sync': 'true',
        }.items())

    return LaunchDescription([
        # arguments
        declare_x,
        declare_y,
        declare_yaw,
        declare_use_sim_time,
        # environment
        set_resource_path_local,
        set_resource_path_tb4,
        # simulation
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        # mapping
        slam,
    ])
